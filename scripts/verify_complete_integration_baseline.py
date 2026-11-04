"""Read-only compatibility guard. Run from any directory; never fetches/writes."""
import hashlib
import json
from pathlib import Path
import sys

root = Path(__file__).resolve().parents[1]
manifest = json.loads((root/'COMPLETE_INTEGRATION_BASELINE.json').read_text())
failed = False
for relative, expected in manifest['git_blob_sha1'].items():
    path = root/relative
    if not path.is_file():
        print('MISSING:',relative)
        failed = True
        continue
    data = path.read_bytes()
    actual = hashlib.sha1(b'blob '+str(len(data)).encode()+b'\0'+data).hexdigest()
    if actual != expected:
        print('CHANGED:',relative,'-- review compatibility; do not overwrite teammate code')
        failed = True
    else:
        print('MATCH:',relative)
sys.exit(1 if failed else 0)
