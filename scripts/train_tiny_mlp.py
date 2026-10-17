#!/usr/bin/env python3
import argparse
import csv
import random
from pathlib import Path

import matplotlib
matplotlib.use("Agg")
import matplotlib.pyplot as plt
import numpy as np
import torch
from torch import nn
from torch.utils.data import DataLoader, TensorDataset


class TinySearchMLP(nn.Module):
    def __init__(self, input_size):
        super().__init__()
        self.network = nn.Sequential(
            nn.Linear(input_size, 32),
            nn.ReLU(),
            nn.Linear(32, 16),
            nn.ReLU(),
            nn.Linear(16, 1),
        )

    def forward(self, inputs):
        return self.network(inputs).squeeze(-1)


def set_seed(seed):
    random.seed(seed)
    np.random.seed(seed)
    torch.manual_seed(seed)
    if torch.cuda.is_available():
        torch.cuda.manual_seed_all(seed)


def group_states(features, targets, state_ids, map_ids):
    unique_state_ids = np.unique(state_ids)
    candidate_counts = np.asarray([
        np.count_nonzero(state_ids == state_id)
        for state_id in unique_state_ids
    ], dtype=np.int32)
    maximum_candidates = int(np.max(candidate_counts))

    grouped_features = np.zeros(
        (unique_state_ids.size, maximum_candidates, features.shape[1]),
        dtype=np.float32,
    )
    grouped_targets = np.zeros(
        (unique_state_ids.size, maximum_candidates),
        dtype=np.float32,
    )
    grouped_valid = np.zeros(
        (unique_state_ids.size, maximum_candidates),
        dtype=bool,
    )
    grouped_maps = np.empty(unique_state_ids.size, dtype=np.int64)

    for group_index, state_id in enumerate(unique_state_ids):
        mask = state_ids == state_id
        state_features = features[mask]
        state_targets = targets[mask]
        state_maps = np.unique(map_ids[mask])

        if state_maps.size != 1:
            raise ValueError(f"State {state_id} belongs to multiple maps")

        count = state_features.shape[0]
        grouped_features[group_index, :count] = state_features
        grouped_targets[group_index, :count] = state_targets
        grouped_valid[group_index, :count] = True
        grouped_maps[group_index] = int(state_maps[0])

    return (
        grouped_features,
        grouped_targets,
        grouped_valid,
        grouped_maps,
        unique_state_ids.astype(np.int64),
        candidate_counts,
    )


def split_maps(unique_maps, seed):
    if unique_maps.size < 5:
        raise ValueError("At least five maps are required for this smoke test")

    rng = np.random.default_rng(seed)
    shuffled = unique_maps.copy()
    rng.shuffle(shuffled)

    test_count = max(1, int(round(0.2 * shuffled.size)))
    validation_count = max(1, int(round(0.2 * shuffled.size)))

    if test_count + validation_count >= shuffled.size:
        validation_count = 1
        test_count = 1

    test_maps = shuffled[:test_count]
    validation_maps = shuffled[
        test_count:test_count + validation_count
    ]
    training_maps = shuffled[test_count + validation_count:]

    return training_maps, validation_maps, test_maps


def make_loader(x, y, valid, batch_size, shuffle):
    dataset = TensorDataset(
        torch.from_numpy(x),
        torch.from_numpy(y),
        torch.from_numpy(valid),
    )
    return DataLoader(
        dataset,
        batch_size=batch_size,
        shuffle=shuffle,
        num_workers=0,
    )


def loss_function(
    logits,
    targets,
    valid,
    teacher_temperature,
    mse_weight,
):
    masked_logits = logits.masked_fill(~valid, -1.0e9)
    teacher_logits = (targets / teacher_temperature).masked_fill(
        ~valid,
        -1.0e9,
    )
    teacher_distribution = torch.softmax(
        teacher_logits,
        dim=1,
    )
    ranking_loss = -(
        teacher_distribution
        * torch.log_softmax(masked_logits, dim=1)
    ).sum(dim=1).mean()

    squared_error = (torch.sigmoid(logits) - targets) ** 2
    score_loss = (
        squared_error * valid.to(squared_error.dtype)
    ).sum() / torch.clamp(valid.sum(), min=1)

    total_loss = ranking_loss + mse_weight * score_loss
    return total_loss, ranking_loss, score_loss


@torch.no_grad()
def evaluate(model, loader, device, teacher_temperature, mse_weight):
    model.eval()
    total_loss = 0.0
    total_states = 0
    top1_correct = 0
    top3_correct = 0
    total_regret = 0.0

    for inputs, targets, valid in loader:
        inputs = inputs.to(device)
        targets = targets.to(device)
        valid = valid.to(device)
        logits = model(inputs)
        loss, _, _ = loss_function(
            logits,
            targets,
            valid,
            teacher_temperature,
            mse_weight,
        )

        batch_size = inputs.shape[0]
        total_loss += float(loss.item()) * batch_size
        total_states += batch_size

        masked_targets = targets.masked_fill(~valid, -1.0e9)
        masked_logits = logits.masked_fill(~valid, -1.0e9)
        expert_best = torch.argmax(masked_targets, dim=1)
        model_best = torch.argmax(masked_logits, dim=1)
        top1_correct += int((expert_best == model_best).sum().item())

        top3 = torch.topk(
            masked_logits,
            k=min(3, logits.shape[1]),
            dim=1,
        ).indices
        top3_correct += int(
            (top3 == expert_best[:, None]).any(dim=1).sum().item()
        )

        best_scores = torch.max(targets, dim=1).values
        selected_scores = targets.gather(
            1,
            model_best[:, None],
        ).squeeze(1)
        total_regret += float(
            (best_scores - selected_scores).sum().item()
        )

    return {
        "loss": total_loss / max(total_states, 1),
        "top1": top1_correct / max(total_states, 1),
        "top3": top3_correct / max(total_states, 1),
        "regret": total_regret / max(total_states, 1),
        "states": total_states,
    }


def export_numpy(model, feature_mean, feature_std, feature_names, path):
    layers = [module for module in model.network if isinstance(module, nn.Linear)]
    np.savez_compressed(
        path,
        feature_mean=feature_mean.astype(np.float32),
        feature_std=feature_std.astype(np.float32),
        feature_names=feature_names,
        weight_1=layers[0].weight.detach().cpu().numpy().astype(np.float32),
        bias_1=layers[0].bias.detach().cpu().numpy().astype(np.float32),
        weight_2=layers[1].weight.detach().cpu().numpy().astype(np.float32),
        bias_2=layers[1].bias.detach().cpu().numpy().astype(np.float32),
        weight_3=layers[2].weight.detach().cpu().numpy().astype(np.float32),
        bias_3=layers[2].bias.detach().cpu().numpy().astype(np.float32),
    )


def verify_numpy_export(model, sample, feature_mean, feature_std, export_path):
    model.eval()
    normalized = (sample - feature_mean) / feature_std

    with torch.no_grad():
        torch_output = model(
            torch.from_numpy(normalized.astype(np.float32))
        ).cpu().numpy()

    with np.load(export_path, allow_pickle=False) as weights:
        hidden_1 = np.maximum(
            normalized @ weights["weight_1"].T + weights["bias_1"],
            0.0,
        )
        hidden_2 = np.maximum(
            hidden_1 @ weights["weight_2"].T + weights["bias_2"],
            0.0,
        )
        numpy_output = (
            hidden_2 @ weights["weight_3"].T + weights["bias_3"]
        ).squeeze(-1)

    maximum_error = float(np.max(np.abs(torch_output - numpy_output)))
    return maximum_error


def main():
    parser = argparse.ArgumentParser()
    parser.add_argument(
        "--dataset",
        type=Path,
        default=Path(
            "~/ee3033_ws/src/ee3033_search_policy/"
            "outputs/training_maps/expert_dataset.npz"
        ).expanduser(),
    )
    parser.add_argument("--epochs", type=int, default=300)
    parser.add_argument("--batch-size", type=int, default=64)
    parser.add_argument("--learning-rate", type=float, default=1e-3)
    parser.add_argument("--weight-decay", type=float, default=1e-4)
    parser.add_argument("--teacher-temperature", type=float, default=0.15)
    parser.add_argument("--mse-weight", type=float, default=0.2)
    parser.add_argument("--patience", type=int, default=40)
    parser.add_argument("--seed", type=int, default=3033)
    args = parser.parse_args()

    set_seed(args.seed)

    with np.load(args.dataset, allow_pickle=False) as data:
        features = data["features"].astype(np.float32)
        targets = data["targets"].astype(np.float32)
        state_ids = data["state_ids"].astype(np.int64)
        map_ids = data["map_ids"].astype(np.int64)
        map_names = data["map_names"]
        feature_names = data["feature_names"]

    (
        state_x,
        state_y,
        state_valid,
        state_maps,
        grouped_state_ids,
        candidate_counts,
    ) = group_states(
        features,
        targets,
        state_ids,
        map_ids,
    )

    training_maps, validation_maps, test_maps = split_maps(
        np.unique(state_maps),
        args.seed,
    )

    train_mask = np.isin(state_maps, training_maps)
    validation_mask = np.isin(state_maps, validation_maps)
    test_mask = np.isin(state_maps, test_maps)

    training_candidates = state_x[
        train_mask[:, None] & state_valid
    ]
    feature_mean = training_candidates.mean(axis=0)
    feature_std = training_candidates.std(axis=0)
    feature_std = np.maximum(feature_std, 1e-6)

    normalized_x = (state_x - feature_mean) / feature_std
    normalized_x[~state_valid] = 0.0

    train_loader = make_loader(
        normalized_x[train_mask],
        state_y[train_mask],
        state_valid[train_mask],
        args.batch_size,
        True,
    )
    validation_loader = make_loader(
        normalized_x[validation_mask],
        state_y[validation_mask],
        state_valid[validation_mask],
        args.batch_size,
        False,
    )
    test_loader = make_loader(
        normalized_x[test_mask],
        state_y[test_mask],
        state_valid[test_mask],
        args.batch_size,
        False,
    )

    device = torch.device("cuda" if torch.cuda.is_available() else "cpu")
    model = TinySearchMLP(state_x.shape[-1]).to(device)
    optimizer = torch.optim.AdamW(
        model.parameters(),
        lr=args.learning_rate,
        weight_decay=args.weight_decay,
    )

    parameter_count = sum(
        parameter.numel() for parameter in model.parameters()
    )

    print("===== TINY MLP TRAINING =====")
    print("Python PyTorch:", torch.__version__)
    print("Device:", device)
    print("Parameters:", parameter_count)
    print("States:", state_x.shape[0])
    print(
        "Candidates per state: "
        f"{int(candidate_counts.min())} to "
        f"{int(candidate_counts.max())}"
    )
    print("Features:", state_x.shape[2])
    print("Training maps:", [str(map_names[i]) for i in training_maps])
    print("Validation maps:", [str(map_names[i]) for i in validation_maps])
    print("Test maps:", [str(map_names[i]) for i in test_maps])

    output_dir = args.dataset.parent / "tiny_mlp"
    output_dir.mkdir(parents=True, exist_ok=True)

    best_path = output_dir / "best_model.pt"
    history = []
    best_validation_loss = float("inf")
    epochs_without_improvement = 0

    for epoch in range(1, args.epochs + 1):
        model.train()
        running_loss = 0.0
        state_count = 0

        for inputs, batch_targets, valid in train_loader:
            inputs = inputs.to(device)
            batch_targets = batch_targets.to(device)
            valid = valid.to(device)

            optimizer.zero_grad(set_to_none=True)
            logits = model(inputs)
            loss, _, _ = loss_function(
                logits,
                batch_targets,
                valid,
                args.teacher_temperature,
                args.mse_weight,
            )
            loss.backward()
            optimizer.step()

            batch_states = inputs.shape[0]
            running_loss += float(loss.item()) * batch_states
            state_count += batch_states

        training_loss = running_loss / max(state_count, 1)
        validation_metrics = evaluate(
            model,
            validation_loader,
            device,
            args.teacher_temperature,
            args.mse_weight,
        )
        history.append((
            epoch,
            training_loss,
            validation_metrics["loss"],
            validation_metrics["top1"],
            validation_metrics["top3"],
            validation_metrics["regret"],
        ))

        if validation_metrics["loss"] < best_validation_loss - 1e-6:
            best_validation_loss = validation_metrics["loss"]
            epochs_without_improvement = 0
            torch.save(model.state_dict(), best_path)
        else:
            epochs_without_improvement += 1

        if epoch == 1 or epoch % 25 == 0:
            print(
                f"Epoch {epoch:03d} | "
                f"train loss {training_loss:.4f} | "
                f"val loss {validation_metrics['loss']:.4f} | "
                f"top1 {validation_metrics['top1'] * 100:.1f}% | "
                f"top3 {validation_metrics['top3'] * 100:.1f}% | "
                f"regret {validation_metrics['regret']:.4f}"
            )

        if epochs_without_improvement >= args.patience:
            print(f"Early stopping at epoch {epoch}")
            break

    model.load_state_dict(
        torch.load(best_path, map_location=device, weights_only=True)
    )

    train_metrics = evaluate(
        model,
        train_loader,
        device,
        args.teacher_temperature,
        args.mse_weight,
    )
    validation_metrics = evaluate(
        model,
        validation_loader,
        device,
        args.teacher_temperature,
        args.mse_weight,
    )
    test_metrics = evaluate(
        model,
        test_loader,
        device,
        args.teacher_temperature,
        args.mse_weight,
    )

    checkpoint_path = output_dir / "tiny_search_policy.pt"
    torch.save({
        "model_state_dict": model.state_dict(),
        "feature_mean": feature_mean,
        "feature_std": feature_std,
        "feature_names": feature_names,
        "training_maps": training_maps,
        "validation_maps": validation_maps,
        "test_maps": test_maps,
        "seed": args.seed,
    }, checkpoint_path)

    numpy_path = output_dir / "tiny_search_policy_weights.npz"
    cpu_model = model.cpu()
    export_numpy(
        cpu_model,
        feature_mean,
        feature_std,
        feature_names,
        numpy_path,
    )
    export_error = verify_numpy_export(
        cpu_model,
        state_x[test_mask][0],
        feature_mean,
        feature_std,
        numpy_path,
    )

    history_path = output_dir / "training_history.csv"
    with history_path.open("w", newline="", encoding="utf-8") as file:
        writer = csv.writer(file)
        writer.writerow([
            "epoch",
            "train_loss",
            "validation_loss",
            "validation_top1",
            "validation_top3",
            "validation_regret",
        ])
        writer.writerows(history)

    epochs = [row[0] for row in history]
    train_losses = [row[1] for row in history]
    validation_losses = [row[2] for row in history]

    plt.figure(figsize=(7.0, 4.5))
    plt.plot(epochs, train_losses, label="Training loss")
    plt.plot(epochs, validation_losses, label="Validation loss")
    plt.xlabel("Epoch")
    plt.ylabel("Loss")
    plt.title("Tiny search-policy MLP training")
    plt.grid(alpha=0.3)
    plt.legend()
    plt.tight_layout()
    figure_path = output_dir / "training_curve.png"
    plt.savefig(figure_path, dpi=180)
    plt.close()

    print("\n===== FINAL METRICS =====")
    for name, metrics in (
        ("Train", train_metrics),
        ("Validation", validation_metrics),
        ("Test", test_metrics),
    ):
        print(
            f"{name:10s} | states={metrics['states']:3d} | "
            f"top1={metrics['top1'] * 100:6.2f}% | "
            f"top3={metrics['top3'] * 100:6.2f}% | "
            f"regret={metrics['regret']:.5f}"
        )

    print("\n===== EXPORT =====")
    print("PyTorch checkpoint:", checkpoint_path)
    print("NumPy weights:", numpy_path)
    print("Training curve:", figure_path)
    print("NumPy/PyTorch maximum error:", f"{export_error:.3e}")


if __name__ == "__main__":
    main()
