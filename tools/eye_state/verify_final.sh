#!/usr/bin/env bash
# 저장된 최종 모델을 학습 없이 다시 평가하고 시험 이미지를 추론한다.
set -euo pipefail

repo_dir="$(cd "$(dirname "${BASH_SOURCE[0]}")/../.." && pwd)"
dataset_dir="$repo_dir/data/eye_state"
output_dir="${EYE_STATE_OUTPUT_DIR:-$HOME/eye_state_runs}"
run_dir="$output_dir/eye_state/baseline"
model_path="$run_dir/weights/eye_state_best.pt"

command -v docker >/dev/null || { echo '오류: docker 명령이 없습니다.' >&2; exit 2; }
docker ps >/dev/null || { echo '오류: 이 셸에서 Docker daemon에 연결할 수 없습니다.' >&2; exit 2; }
[[ -f "$model_path" ]] || { echo "오류: 최종 모델이 없습니다: $model_path" >&2; exit 2; }
[[ -d "$dataset_dir/test/open" && -d "$dataset_dir/test/closed" ]] || { echo '오류: 시험 데이터셋 구조가 없습니다.' >&2; exit 2; }

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

eye_docker 'python3 /workspace/tools/eye_state/evaluate_eye_classifier.py --model /outputs/eye_state/baseline/weights/eye_state_best.pt --data /dataset --device 0 --imgsz 224 --output /outputs/eye_state/baseline/final_misclassified.csv --all-output /outputs/eye_state/baseline/final_predictions.csv' | tee "$run_dir/final_evaluation.log"

open_image="$(find "$dataset_dir/test/open" -maxdepth 1 -type f -name '*.jpg' -print -quit)"
closed_image="$(find "$dataset_dir/test/closed" -maxdepth 1 -type f -name '*.jpg' -print -quit)"
[[ -n "$open_image" && -n "$closed_image" ]] || { echo '오류: OPEN 또는 CLOSED 시험 이미지가 없습니다.' >&2; exit 2; }
open_image="/dataset/test/open/$(basename "$open_image")"
closed_image="/dataset/test/closed/$(basename "$closed_image")"
eye_docker "python3 /workspace/tools/eye_state/infer_eye_classifier.py --model /outputs/eye_state/baseline/weights/eye_state_best.pt --image '$open_image' --device 0 --imgsz 224" | tee "$run_dir/final_open_inference.log"
eye_docker "python3 /workspace/tools/eye_state/infer_eye_classifier.py --model /outputs/eye_state/baseline/weights/eye_state_best.pt --image '$closed_image' --device 0 --imgsz 224" | tee "$run_dir/final_closed_inference.log"

lowest_confidence_image="$(python3 - "$run_dir/final_predictions.csv" <<'PY'
import csv
import sys

with open(sys.argv[1], newline='', encoding='utf-8') as stream:
    row = min(csv.DictReader(stream), key=lambda item: float(item['confidence']))
print(row['path'])
PY
)"
eye_docker "python3 /workspace/tools/eye_state/infer_eye_classifier.py --model /outputs/eye_state/baseline/weights/eye_state_best.pt --image '$lowest_confidence_image' --device 0 --imgsz 224" | tee "$run_dir/final_lowest_confidence_inference.log"

echo "재검증 완료: $run_dir"
