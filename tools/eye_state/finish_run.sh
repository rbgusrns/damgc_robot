#!/usr/bin/env bash
# 중단된 Eye State 학습을 재개하고 독립 시험·추론을 실행한다.
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
dataset_dir="$repo_dir/data/eye_state"
output_dir="${EYE_STATE_OUTPUT_DIR:-$HOME/eye_state_runs}"
run_dir="$output_dir/eye_state/baseline"
last_weight="$run_dir/weights/last.pt"
best_weight="$run_dir/weights/best.pt"

command -v docker >/dev/null || { echo '오류: docker 명령을 찾을 수 없습니다.' >&2; exit 2; }
docker ps >/dev/null || { echo '오류: 이 셸에서 Docker daemon에 연결할 수 없습니다.' >&2; exit 2; }
if [[ -n "$(docker ps -q)" ]]; then
  docker ps --format '{{.Names}}  {{.Image}}  {{.Status}}' >&2
  if [[ "${EYE_STATE_ALLOW_ACTIVE_CONTAINERS:-0}" != 1 ]]; then
    echo '오류: 실행 중인 컨테이너가 있습니다. 동시 학습을 명시적으로 선택하려면 EYE_STATE_ALLOW_ACTIVE_CONTAINERS=1을 사용하세요.' >&2
    exit 2
  fi
  echo '경고: 실행 중인 컨테이너와 GPU·메모리를 공유하며 학습합니다. 로봇 런타임의 지연이나 메모리 부족이 발생할 수 있습니다.' >&2
fi
available_kib="$(awk '/^MemAvailable:/ {print $2}' /proc/meminfo)"
echo "호스트 사용 가능 메모리: $((available_kib / 1024)) MiB"
if (( available_kib < 2 * 1024 * 1024 )); then
  echo '오류: 호스트 사용 가능 메모리가 2 GiB 미만입니다. 학습을 시작하지 않습니다.' >&2
  exit 2
fi
[[ -d "$dataset_dir" ]] || { echo "오류: 데이터셋이 없습니다: $dataset_dir" >&2; exit 2; }
[[ -f "$last_weight" ]] || { echo "오류: 재개 체크포인트가 없습니다: $last_weight" >&2; exit 2; }
mkdir -p "$run_dir"

eye_docker() {
  docker run --rm --runtime=nvidia --ipc=host \
    --user "$(id -u):$(id -g)" \
    -e NVIDIA_VISIBLE_DEVICES=all \
    -e NVIDIA_DRIVER_CAPABILITIES=compute,utility \
    -e YOLO_CONFIG_DIR=/tmp \
    -v "$repo_dir:/workspace:ro" \
    -v "$dataset_dir:/dataset:ro" \
    -v "$output_dir:/outputs" \
    --workdir /outputs \
    damgc-survivor-yolo:humble "$1"
}

eye_docker 'python3 /workspace/tools/eye_state/validate_dataset.py --data /dataset' | tee "$run_dir/final_validation.log"

completed_epochs="$(awk -F, 'NR>1 {epoch=$1} END {print epoch+0}' "$run_dir/results.csv")"
if (( completed_epochs < 40 )); then
  echo "기록된 학습: ${completed_epochs}/40 epoch. last.pt에서 재개합니다."
  eye_docker 'python3 /workspace/tools/eye_state/train_eye_classifier.py --data /dataset --model /outputs/eye_state/baseline/weights/last.pt --device 0 --project /outputs/eye_state --name baseline --resume' 2>&1 | tee "$run_dir/resume.log"
fi

completed_epochs="$(awk -F, 'NR>1 {epoch=$1} END {print epoch+0}' "$run_dir/results.csv")"
if (( completed_epochs < 40 )); then
  echo "오류: 학습이 완료되지 않았습니다. 현재 ${completed_epochs}/40 epoch입니다." >&2
  exit 2
fi
[[ -f "$best_weight" ]] || { echo "오류: best.pt가 없습니다: $best_weight" >&2; exit 2; }

eye_docker 'python3 /workspace/tools/eye_state/evaluate_eye_classifier.py --model /outputs/eye_state/baseline/weights/best.pt --data /dataset --device 0 --imgsz 224 --output /outputs/eye_state/baseline/test_misclassified.csv --all-output /outputs/eye_state/baseline/test_predictions.csv' | tee "$run_dir/test_evaluation.log"

open_image="$(find "$dataset_dir/test/open" -maxdepth 1 -type f -name '*.jpg' -print -quit)"
closed_image="$(find "$dataset_dir/test/closed" -maxdepth 1 -type f -name '*.jpg' -print -quit)"
[[ -n "$open_image" && -n "$closed_image" ]] || { echo '오류: OPEN/CLOSED 시험 이미지가 없습니다.' >&2; exit 2; }
open_image="/dataset/test/open/$(basename "$open_image")"
closed_image="/dataset/test/closed/$(basename "$closed_image")"
eye_docker "python3 /workspace/tools/eye_state/infer_eye_classifier.py --model /outputs/eye_state/baseline/weights/best.pt --image '$open_image' --device 0 --imgsz 224" | tee "$run_dir/open_inference.log"
eye_docker "python3 /workspace/tools/eye_state/infer_eye_classifier.py --model /outputs/eye_state/baseline/weights/best.pt --image '$closed_image' --device 0 --imgsz 224" | tee "$run_dir/closed_inference.log"

lowest_confidence_image="$(python3 - "$run_dir/test_predictions.csv" <<'PY'
import csv
import sys

with open(sys.argv[1], newline='', encoding='utf-8') as stream:
    row = min(csv.DictReader(stream), key=lambda item: float(item['confidence']))
print(row['path'])
PY
)"
eye_docker "python3 /workspace/tools/eye_state/infer_eye_classifier.py --model /outputs/eye_state/baseline/weights/best.pt --image '$lowest_confidence_image' --device 0 --imgsz 224" | tee "$run_dir/lowest_confidence_inference.log"

named_weight="$run_dir/weights/eye_state_best.pt"
if [[ -e "$named_weight" ]]; then
  cmp -s "$best_weight" "$named_weight" || { echo "오류: 기존 모델 파일이 현재 best.pt와 다릅니다: $named_weight" >&2; exit 2; }
else
  cp "$best_weight" "$named_weight"
fi
echo "완료: $run_dir"
