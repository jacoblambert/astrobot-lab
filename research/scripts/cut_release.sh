#!/bin/bash
# Cut a release on main from the current dev branch without research/ (run by the maintainer).
#   research/scripts/cut_release.sh v0.1.0            # commit + annotated tag on main
#   PUSH=1 research/scripts/cut_release.sh v0.1.0     # also push main and the tag
# main keeps its own history: each release is one commit whose tree is dev's tree minus research/.
set -euo pipefail
cd "$(dirname "$0")/../.."
VERSION=${1:?usage: $0 <version tag, e.g. v0.1.0>}
PUSH=${PUSH:-0}
DEV=$(git rev-parse --abbrev-ref HEAD)
[ "$DEV" != main ] || { echo "run from the dev branch" >&2; exit 2; }
git diff --quiet && git diff --cached --quiet || { echo "commit or stash dev changes first" >&2; exit 2; }
git rev-parse -q --verify "refs/tags/$VERSION" >/dev/null && { echo "tag $VERSION exists" >&2; exit 2; }
DEV_COMMIT=$(git rev-parse --short HEAD)

git checkout main
git read-tree -u --reset "$DEV"            # main's index and work tree become dev's tree
git rm -r -q --cached research && rm -rf research
git diff --cached --stat | tail -3
read -rp "Release $VERSION from $DEV ($DEV_COMMIT) staged on main -- Enter to commit, Ctrl-C to abort "
git commit -m "Astrobot $VERSION

Release cut from $DEV at $DEV_COMMIT without research/."
git tag -a "$VERSION" -m "Astrobot $VERSION"
if [ "$PUSH" = 1 ]; then git push origin main "$VERSION"; fi
git checkout "$DEV"
echo "Released $VERSION on main; back on $DEV."
