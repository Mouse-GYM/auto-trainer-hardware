import re
import subprocess
import sys
from pathlib import Path

# Resolve the repository independently of the directory from which CMake invokes this script.
repo_root = Path(__file__).resolve().parent.parent

try:
    cmd = subprocess.run(
        ["git", "describe", "--tags", "--always", "--dirty"],
        capture_output=True,
        check=True,
        cwd=repo_root,
        text=True,
    )
except FileNotFoundError as exc:
    print(f"gen_version.py: unable to run git: {exc}", file=sys.stderr)
    raise SystemExit(1) from exc
except subprocess.CalledProcessError as exc:
    detail = exc.stderr.strip() or f"git exited with status {exc.returncode}"
    print(f"gen_version.py: git describe failed: {detail}", file=sys.stderr)
    raise SystemExit(exc.returncode) from exc

git_describe = cmd.stdout.strip()

major = 0
minor = 0
patch = 0
tweak = 0
extraversion = ""

# Use a regular expression to extract the version number
# Example match: v1.1.1-6-g59ac656-dirty
m1 = re.match(r'v(\d+)\.(\d+)\.(\d+)(.*)', git_describe)
if m1:
    major, minor, patch, more_info = m1.groups()

    if more_info:
        m2 = re.match(r'-(\d+)-(.*)', more_info)
        if m2:
            tweak, extraversion = m2.groups()
            extraversion = extraversion.replace('-', '')

print(f"VERSION_MAJOR = {major}")
print(f"VERSION_MINOR = {minor}")
print(f"PATCHLEVEL = {patch}")
print(f"VERSION_TWEAK = {tweak}")
print(f"EXTRAVERSION = {extraversion}")
