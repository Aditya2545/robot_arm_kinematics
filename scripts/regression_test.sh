#!/usr/bin/env bash
# =============================================================================
# regression_test.sh — IK→FK roundtrip regression harness
# =============================================================================
set -euo pipefail

CLI="${1:-$(dirname "$0")/../build/bin/rak_cli}"

if [[ ! -x "$CLI" ]]; then
    echo "[ERROR] rak_cli not found at: $CLI"
    echo "        Build first with:  cmake --build build"
    exit 1
fi

PASS=0
FAIL=0
EPSILON_POS=0.001
EPSILON_ANG=0.001

RED='\033[0;31m'; GREEN='\033[0;32m'; RESET='\033[0m'

run_roundtrip() {
    local name="$1"; shift
    local q=("$@")

    # --- FK ---
    fk_out=$("$CLI" fk "${q[@]}" 2>&1)
    # Line:  "  Pose   : pos=(x, y, z) m  rpy=(r, p, y) rad"
    pose_line=$(echo "$fk_out" | grep "Pose")
    fx=$(echo "$pose_line" | python3 -c "import sys,re; m=re.search(r'pos=\((.*?),(.*?),(.*?)\)',sys.stdin.read()); print(m.group(1).strip())")
    fy=$(echo "$pose_line" | python3 -c "import sys,re; m=re.search(r'pos=\((.*?),(.*?),(.*?)\)',sys.stdin.read()); print(m.group(2).strip())")
    fz=$(echo "$pose_line" | python3 -c "import sys,re; m=re.search(r'pos=\((.*?),(.*?),(.*?)\)',sys.stdin.read()); print(m.group(3).strip())")
    fr=$(echo "$pose_line" | python3 -c "import sys,re; m=re.search(r'rpy=\((.*?),(.*?),(.*?)\)',sys.stdin.read()); print(m.group(1).strip())")
    fp=$(echo "$pose_line" | python3 -c "import sys,re; m=re.search(r'rpy=\((.*?),(.*?),(.*?)\)',sys.stdin.read()); print(m.group(2).strip())")
    fy2=$(echo "$pose_line" | python3 -c "import sys,re; m=re.search(r'rpy=\((.*?),(.*?),(.*?)\)',sys.stdin.read()); print(m.group(3).strip())")

    # --- IK ---
    ik_out=$("$CLI" ik "$fx" "$fy" "$fz" "$fr" "$fp" "$fy2" 2>&1) || {
        printf "${RED}FAIL${RESET} [%s]  IK solver did not converge\n" "$name"
        FAIL=$((FAIL+1)); return
    }

    ach_line=$(echo "$ik_out" | grep "Achieved")
    ax=$(echo "$ach_line" | python3 -c "import sys,re; m=re.search(r'pos=\((.*?),(.*?),(.*?)\)',sys.stdin.read()); print(m.group(1).strip())")
    ay=$(echo "$ach_line" | python3 -c "import sys,re; m=re.search(r'pos=\((.*?),(.*?),(.*?)\)',sys.stdin.read()); print(m.group(2).strip())")
    az=$(echo "$ach_line" | python3 -c "import sys,re; m=re.search(r'pos=\((.*?),(.*?),(.*?)\)',sys.stdin.read()); print(m.group(3).strip())")
    ar=$(echo "$ach_line" | python3 -c "import sys,re; m=re.search(r'rpy=\((.*?),(.*?),(.*?)\)',sys.stdin.read()); print(m.group(1).strip())")
    ap=$(echo "$ach_line" | python3 -c "import sys,re; m=re.search(r'rpy=\((.*?),(.*?),(.*?)\)',sys.stdin.read()); print(m.group(2).strip())")
    ay2=$(echo "$ach_line" | python3 -c "import sys,re; m=re.search(r'rpy=\((.*?),(.*?),(.*?)\)',sys.stdin.read()); print(m.group(3).strip())")

    # --- Error ---
    result=$(python3 -c "
import math
dx=$ax-($fx); dy=$ay-($fy); dz=$az-($fz)
pos_err=math.sqrt(dx*dx+dy*dy+dz*dz)
def da(a,b):
    d=a-b
    while d> math.pi: d-=2*math.pi
    while d<-math.pi: d+=2*math.pi
    return abs(d)
ang_err=max(da($ar,$fr),da($ap,$fp),da($ay2,$fy2))
ok='PASS' if pos_err<$EPSILON_POS and ang_err<$EPSILON_ANG else 'FAIL'
print(ok, pos_err, ang_err)
")
    status=$(echo "$result" | awk '{print $1}')
    pe=$(echo "$result"    | awk '{print $2}')
    ae=$(echo "$result"    | awk '{print $3}')

    if [[ "$status" == "PASS" ]]; then
        printf "${GREEN}PASS${RESET} [%s]  pos_err=%s  ang_err=%s\n" "$name" "$pe" "$ae"
        PASS=$((PASS+1))
    else
        printf "${RED}FAIL${RESET} [%s]  pos_err=%s (lim=%s)  ang_err=%s (lim=%s)\n" \
            "$name" "$pe" "$EPSILON_POS" "$ae" "$EPSILON_ANG"
        FAIL=$((FAIL+1))
    fi
}

echo "============================================="
echo " RAK Regression Test Suite  (eps_pos=${EPSILON_POS}m  eps_ang=${EPSILON_ANG}rad)"
echo "============================================="
echo ""

run_roundtrip "Home"         0.0  0.0   0.0   0.0  0.0  0.0
run_roundtrip "Elbow-up"     0.5 -1.0   1.2  -0.5  0.3 -0.2
run_roundtrip "Wrist-bent"   0.0 -1.57  1.57 -1.57 1.57 0.0
run_roundtrip "Random-1"    -0.8  0.4  -1.1   0.7 -0.3  1.5
run_roundtrip "Quarter-turn" 1.57 0.0   0.0   0.0  0.0  0.0
run_roundtrip "Full-config"  0.3 -0.7   0.9  -1.1  0.5 -0.4

echo ""
echo "============================================="
printf " Results:  PASS=%d  FAIL=%d\n" "$PASS" "$FAIL"
echo "============================================="

[[ $FAIL -eq 0 ]]
