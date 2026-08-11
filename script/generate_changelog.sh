#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
OUTPUT_FILE="${1:-${REPO_ROOT}/CHANGELOG.md}"
MAX_COUNT="${MAX_COUNT:-50}"
PACKAGE_NAME="${PACKAGE_NAME:-modi_canevo_sdk}"

if [[ ! "${MAX_COUNT}" =~ ^[1-9][0-9]*$ ]]; then
  echo "error: MAX_COUNT must be a positive integer" >&2
  exit 1
fi
if [[ ! -f "${REPO_ROOT}/version" ]]; then
  echo "error: version file not found: ${REPO_ROOT}/version" >&2
  exit 1
fi
if ! git -C "${REPO_ROOT}" rev-parse --is-inside-work-tree >/dev/null 2>&1; then
  echo "error: not a Git repository: ${REPO_ROOT}" >&2
  exit 1
fi

VERSION="$(tr -d '[:space:]' < "${REPO_ROOT}/version")"
if [[ -z "${VERSION}" ]]; then
  echo "error: version file is empty: ${REPO_ROOT}/version" >&2
  exit 1
fi

COMMIT="$(git -C "${REPO_ROOT}" rev-parse --short HEAD)"
RELEASE_DATE="$(date +%Y-%m-%d)"
HOST_PLATFORM="$(uname -s | tr '[:upper:]' '[:lower:]')"
HOST_ARCH="$(uname -m)"
case "${HOST_ARCH}" in
  aarch64|arm64) ARCH="arm64" ;;
  x86_64|amd64) ARCH="x86_64" ;;
  *) ARCH="${HOST_ARCH}" ;;
esac

CURRENT_TAG="$(git -C "${REPO_ROOT}" tag --points-at HEAD --sort=-version:refname | head -n 1)"
if [[ -n "${CURRENT_TAG}" ]]; then
  PREVIOUS_TAG="$(git -C "${REPO_ROOT}" describe --tags --abbrev=0 HEAD^ 2>/dev/null || true)"
else
  PREVIOUS_TAG="$(git -C "${REPO_ROOT}" describe --tags --abbrev=0 HEAD 2>/dev/null || true)"
fi

commit_subjects() {
  if [[ -n "${PREVIOUS_TAG}" ]]; then
    git -C "${REPO_ROOT}" log --no-merges -n "${MAX_COUNT}" --format='%s' \
      "${PREVIOUS_TAG}..HEAD"
  else
    git -C "${REPO_ROOT}" log --no-merges -n "${MAX_COUNT}" --format='%s' HEAD
  fi
}

FEATURES="$(commit_subjects | awk 'BEGIN { IGNORECASE = 1 } /^feat(\([^)]*\))?!?:/ { print "- " $0 }')"
FIXES="$(commit_subjects | awk 'BEGIN { IGNORECASE = 1 } /^fix(\([^)]*\))?!?:/ { print "- " $0 }')"

mkdir -p "$(dirname "${OUTPUT_FILE}")"
{
  echo "# Changelog"
  echo
  echo "## ${VERSION} - ${RELEASE_DATE}"
  echo
  echo "### Package"
  echo "- Name: ${PACKAGE_NAME}"
  echo "- Platform: ${HOST_PLATFORM}"
  echo "- Architecture: ${ARCH}"
  echo "- Release commit: ${COMMIT}"
  echo "- Changes since: ${PREVIOUS_TAG:-repository start}"
  echo
  echo "### Features"
  if [[ -n "${FEATURES}" ]]; then
    printf '%s\n' "${FEATURES}"
  else
    echo "- None"
  fi
  echo
  echo "### Fixes"
  if [[ -n "${FIXES}" ]]; then
    printf '%s\n' "${FIXES}"
  else
    echo "- None"
  fi
} > "${OUTPUT_FILE}"

echo "Generated changelog: ${OUTPUT_FILE}"
