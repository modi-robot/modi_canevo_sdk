#!/usr/bin/env bash
set -euo pipefail

SCRIPT_DIR="$(cd "$(dirname "${BASH_SOURCE[0]}")" && pwd)"
REPO_ROOT="$(cd "${SCRIPT_DIR}/.." && pwd)"
BUILD_DIR="${BUILD_DIR:-${REPO_ROOT}/build}"
DIST_DIR="${DIST_DIR:-${BUILD_DIR}/dist}"
BUILD_TYPE="${BUILD_TYPE:-Release}"
BUILD_WORKERS="${BUILD_WORKERS:-$(nproc)}"
CHANGELOG_FILE="${DIST_DIR}/modi_canevo_sdk_CHANGELOG.md"
RELEASE_REMOTE="${RELEASE_REMOTE:-origin}"
TEMP_DIR=""

cleanup() {
  if [[ -n "${TEMP_DIR}" && -d "${TEMP_DIR}" ]]; then
    rm -rf "${TEMP_DIR}"
  fi
}
trap cleanup EXIT

for command_name in cmake git zip unzip; do
  if ! command -v "${command_name}" >/dev/null 2>&1; then
    echo "error: required command not found: ${command_name}" >&2
    exit 1
  fi
done

if [[ ! "${BUILD_WORKERS}" =~ ^[1-9][0-9]*$ ]]; then
  echo "error: BUILD_WORKERS must be a positive integer" >&2
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

if [[ -n "$(git -C "${REPO_ROOT}" status --porcelain)" ]]; then
  echo "error: working tree is dirty; commit all release changes before packaging and tagging" >&2
  git -C "${REPO_ROOT}" status --short >&2
  exit 1
fi
if ! git -C "${REPO_ROOT}" remote get-url "${RELEASE_REMOTE}" >/dev/null 2>&1; then
  echo "error: Git remote not found: ${RELEASE_REMOTE}" >&2
  exit 1
fi

mkdir -p "${BUILD_DIR}" "${DIST_DIR}"

echo "[1/6] Generating changelog..."
"${SCRIPT_DIR}/generate_changelog.sh" "${CHANGELOG_FILE}"

echo "[2/6] Configuring SDK..."
cmake -S "${REPO_ROOT}" -B "${BUILD_DIR}" \
  -DCMAKE_BUILD_TYPE="${BUILD_TYPE}"

echo "[3/6] Building SDK..."
cmake --build "${BUILD_DIR}" --parallel "${BUILD_WORKERS}"

echo "[4/6] Creating CPack archive..."
cmake --build "${BUILD_DIR}" --target package --parallel "${BUILD_WORKERS}"

PACKAGE_BASENAME="$(sed -n 's/^set(CPACK_PACKAGE_FILE_NAME "\(.*\)")/\1/p' \
  "${BUILD_DIR}/CPackConfig.cmake" | head -n 1)"
if [[ -z "${PACKAGE_BASENAME}" ]]; then
  echo "error: unable to read CPACK_PACKAGE_FILE_NAME" >&2
  exit 1
fi

SOURCE_ARCHIVE="${BUILD_DIR}/${PACKAGE_BASENAME}.zip"
DEST_ARCHIVE="${DIST_DIR}/${PACKAGE_BASENAME}.zip"
if [[ ! -f "${SOURCE_ARCHIVE}" ]]; then
  echo "error: CPack archive not found: ${SOURCE_ARCHIVE}" >&2
  exit 1
fi

echo "[5/6] Adding changelog and staging artifacts..."
TEMP_DIR="$(mktemp -d /tmp/modi_canevo_sdk_package.XXXXXX)"
unzip -q "${SOURCE_ARCHIVE}" -d "${TEMP_DIR}"

if [[ -d "${TEMP_DIR}/${PACKAGE_BASENAME}" ]]; then
  cp "${CHANGELOG_FILE}" "${TEMP_DIR}/${PACKAGE_BASENAME}/CHANGELOG.md"
else
  cp "${CHANGELOG_FILE}" "${TEMP_DIR}/CHANGELOG.md"
fi

rm -f "${DEST_ARCHIVE}"
(cd "${TEMP_DIR}" && zip -r -q "${DEST_ARCHIVE}" .)

echo "[6/6] Tagging and pushing ${RELEASE_TAG}..."
if git -C "${REPO_ROOT}" rev-parse -q --verify "refs/tags/${RELEASE_TAG}" >/dev/null; then
  TAG_COMMIT="$(git -C "${REPO_ROOT}" rev-list -n 1 "${RELEASE_TAG}")"
  HEAD_COMMIT="$(git -C "${REPO_ROOT}" rev-parse HEAD)"
  if [[ "${TAG_COMMIT}" != "${HEAD_COMMIT}" ]]; then
    echo "error: tag ${RELEASE_TAG} already exists on another commit" >&2
    exit 1
  fi
  echo "Tag ${RELEASE_TAG} already points to HEAD; reusing it."
else
  git -C "${REPO_ROOT}" tag -a "${RELEASE_TAG}" -m "Release ${VERSION}"
fi
git -C "${REPO_ROOT}" push "${RELEASE_REMOTE}" "refs/tags/${RELEASE_TAG}"

echo "Done:"
echo "  ${DEST_ARCHIVE}"
echo "  ${CHANGELOG_FILE}"
echo "  tag ${RELEASE_TAG} -> ${RELEASE_REMOTE}"
