# Test-only command doubles. PATH is empty: no Docker, GPU or simulator work.
dirname() { printf '%s/docker\n' "$INSPECTION_TEST_ROOT"; }
hostname() { printf 'inspection-test-host\n'; }
xdpyinfo() { return 0; }
nvidia-smi() { return 0; }
python3() { return 0; }
stat() { printf '2049\n'; }
docker() {
  case "$*" in
    inspect\ *) printf 'false\n' ;;
    exec\ astrobot-selected-dev\ stat\ *) printf '2049\n' ;;
    *'ros2 node list'*)
      printf '%s\n' "${INSPECTION_TEST_NODES:-}"
      return "${INSPECTION_TEST_QUERY_EXIT:-0}"
      ;;
    *'up -d --no-deps astrobot-sim') printf 'TEST_SIMULATOR_START\n' ;;
    *) return 0 ;;
  esac
}
