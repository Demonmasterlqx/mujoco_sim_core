#!/usr/bin/env bash
set -uo pipefail

is_server=0
is_wbr=0
ticks=0
while (($# > 0)); do
  case "$1" in
    --model) is_server=1; shift 2 ;;
    --ticks) is_wbr=1; ticks="$2"; shift 2 ;;
    *) shift ;;
  esac
done

if ((is_server == 1)); then
  trap 'exit 0' INT TERM
  echo "event=server_started"
  sleep 0.1
  echo "event=adapter_registered adapter_id=double_inverted_controller"
  echo "event=adapter_activated adapter_id=double_inverted_controller"
  echo "event=adapter_registered adapter_id=wbr_lqr_controller"
  echo "event=adapter_activated adapter_id=wbr_lqr_controller"
  while true; do sleep 1; done
fi

if ((is_wbr == 1)); then
  sleep 0.3
  echo "wbr-lqr summary ticks=${ticks} nonzero_ticks=1 clamps=0 space_effects=1 e_effects=1 q_effects=0 final_fsm=normal"
  exit 0
fi

trap 'exit 0' INT TERM
echo "Command layout:"
while true; do sleep 1; done
