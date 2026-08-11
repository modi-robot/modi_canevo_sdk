#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
DIST_DIR="${DIST_DIR:-${REPO_ROOT}/build/dist}"
GITHUB_REPOSITORY="${GITHUB_REPOSITORY:-modi-robot/modi_canevo_sdk}"
CHANGELOG_FILE="${DIST_DIR}/modi_sdk_CHANGELOG.md"

if ! command -v gh >/dev/null 2>&1; then
  echo "error: GitHub CLI (gh) is required" >&2
  exit 1
fi
if ! gh auth status >/dev/null 2>&1; then
  echo "error: GitHub CLI is not authenticated; run: gh auth login" >&2
  exit 1
fi
if [[ ! -f "${REPO_ROOT}/version" ]]; then
  echo "error: version file not found: ${REPO_ROOT}/version" >&2
  exit 1
fi

VERSION="$(tr -d '[:space:]' < "${REPO_ROOT}/version")"
if [[ ! "${VERSION}" =~ ^[0-9]+\.[0-9]+\.[0-9]+$ ]]; then
  echo "error: version must use x.x.x format, found: ${VERSION:-empty}" >&2
  exit 1
fi

RELEASE_TAG="v${VERSION}"
RELEASE_TITLE="Release ${RELEASE_TAG}"
if ! git -C "${REPO_ROOT}" rev-parse -q --verify "refs/tags/${RELEASE_TAG}" >/dev/null; then
  echo "error: local tag not found: ${RELEASE_TAG}; run package_sdk.sh first" >&2
  exit 1
fi
TAG_COMMIT="$(git -C "${REPO_ROOT}" rev-list -n 1 "${RELEASE_TAG}")"

if [[ ! -f "${CHANGELOG_FILE}" ]]; then
  echo "error: changelog not found: ${CHANGELOG_FILE}" >&2
  exit 1
fi

shopt -s nullglob
ZIP_FILES=("${DIST_DIR}/modi_sdk_${VERSION}_"*.zip)
shopt -u nullglob
if [[ "${#ZIP_FILES[@]}" -ne 1 ]]; then
  echo "error: expected exactly one SDK ${VERSION} zip in ${DIST_DIR}, found ${#ZIP_FILES[@]}" >&2
  exit 1
fi
ZIP_FILE="${ZIP_FILES[0]}"

echo "Repository : ${GITHUB_REPOSITORY}"
echo "Release    : ${RELEASE_TITLE}"
echo "Tag        : ${RELEASE_TAG} (${TAG_COMMIT})"
echo "Attachments:"
echo "  ${ZIP_FILE}"
echo "  ${CHANGELOG_FILE}"

if gh release view "${RELEASE_TAG}" --repo "${GITHUB_REPOSITORY}" >/dev/null 2>&1; then
  echo "Updating existing GitHub Release..."
  gh release edit "${RELEASE_TAG}" \
    --repo "${GITHUB_REPOSITORY}" \
    --title "${RELEASE_TITLE}" \
    --notes-file "${CHANGELOG_FILE}"
  gh release upload "${RELEASE_TAG}" \
    "${ZIP_FILE}" "${CHANGELOG_FILE}" \
    --repo "${GITHUB_REPOSITORY}" \
    --clobber
else
  echo "Creating GitHub Release..."
  gh release create "${RELEASE_TAG}" \
    "${ZIP_FILE}" "${CHANGELOG_FILE}" \
    --repo "${GITHUB_REPOSITORY}" \
    --target "${TAG_COMMIT}" \
    --title "${RELEASE_TITLE}" \
    --notes-file "${CHANGELOG_FILE}"
fi

echo "Published: https://github.com/${GITHUB_REPOSITORY}/releases/tag/${RELEASE_TAG}"
