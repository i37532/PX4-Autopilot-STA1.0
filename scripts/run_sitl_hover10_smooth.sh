#!/usr/bin/env bash
set -euo pipefail

ROOT_DIR=$(cd "$(dirname "${BASH_SOURCE[0]}")/.." && pwd)
LAUNCH_DELAY_S=${LAUNCH_DELAY_S:-15}
OFFBOARD_SCRIPT=${OFFBOARD_SCRIPT:-scripts/offboard_mavsdk_hover10_smooth.py}
LOG_FILE=${LOG_FILE:-"$ROOT_DIR/offboard_mavsdk_hover10_smooth.log"}

command_exists() {
    command -v "$1" >/dev/null 2>&1
}

has_gui() {
    [[ -n "${DISPLAY:-}" || -n "${WAYLAND_DISPLAY:-}" ]]
}

xterm_emulator_is_gnome() {
    local path
    path=$(command -v x-terminal-emulator 2>/dev/null || true)
    if [[ -z "$path" ]]; then
        return 1
    fi
    if command_exists readlink; then
        path=$(readlink -f "$path" 2>/dev/null || echo "$path")
    fi
    [[ "$path" == *gnome-terminal* ]]
}

launch_offboard_terminal() {
    local cmd="cd \"${ROOT_DIR}\"; python3 \"${OFFBOARD_SCRIPT}\""
    local cmd_string
    cmd_string=$(printf '%q' "$cmd")

    if has_gui; then
        if command_exists x-terminal-emulator; then
            if ! xterm_emulator_is_gnome || command_exists dbus-launch; then
                if x-terminal-emulator -e bash -lc "$cmd" >/dev/null 2>&1; then
                    echo "Launched offboard script using x-terminal-emulator."
                    return 0
                fi
            fi
        fi

        if command_exists gnome-terminal && command_exists dbus-launch; then
            if gnome-terminal -- bash -lc "$cmd" >/dev/null 2>&1; then
                echo "Launched offboard script using gnome-terminal."
                return 0
            fi
        fi

        if command_exists konsole; then
            if konsole -e bash -lc "$cmd" >/dev/null 2>&1; then
                echo "Launched offboard script using konsole."
                return 0
            fi
        fi

        if command_exists xfce4-terminal; then
            if xfce4-terminal -e "bash -lc $cmd_string" >/dev/null 2>&1; then
                echo "Launched offboard script using xfce4-terminal."
                return 0
            fi
        fi

        if command_exists mate-terminal; then
            if mate-terminal -- bash -lc "$cmd" >/dev/null 2>&1; then
                echo "Launched offboard script using mate-terminal."
                return 0
            fi
        fi

        if command_exists lxterminal; then
            if lxterminal -e bash -lc "$cmd" >/dev/null 2>&1; then
                echo "Launched offboard script using lxterminal."
                return 0
            fi
        fi

        if command_exists terminator; then
            if terminator -e "bash -lc $cmd_string" >/dev/null 2>&1; then
                echo "Launched offboard script using terminator."
                return 0
            fi
        fi

        if command_exists xterm; then
            if xterm -e bash -lc "$cmd" >/dev/null 2>&1; then
                echo "Launched offboard script using xterm."
                return 0
            fi
        fi

        if command_exists alacritty; then
            if alacritty -e bash -lc "$cmd" >/dev/null 2>&1; then
                echo "Launched offboard script using alacritty."
                return 0
            fi
        fi

        if command_exists kitty; then
            if kitty -- bash -lc "$cmd" >/dev/null 2>&1; then
                echo "Launched offboard script using kitty."
                return 0
            fi
        fi

        if command_exists wezterm; then
            if wezterm start -- bash -lc "$cmd" >/dev/null 2>&1; then
                echo "Launched offboard script using wezterm."
                return 0
            fi
        fi
    fi

    if command_exists tmux; then
        if [[ -n "${TMUX:-}" ]]; then
            tmux new-window -n "px4-offboard" "cd \"$ROOT_DIR\"; python3 \"$OFFBOARD_SCRIPT\""
            echo "Launched offboard script in new tmux window: px4-offboard."
            return 0
        fi
        tmux new-session -d -s px4-offboard "cd \"$ROOT_DIR\"; python3 \"$OFFBOARD_SCRIPT\""
        echo "Launched offboard script in tmux session: px4-offboard (attach with: tmux attach -t px4-offboard)."
        return 0
    fi

    if command_exists screen; then
        screen -dmS px4-offboard bash -lc "cd \"$ROOT_DIR\"; python3 \"$OFFBOARD_SCRIPT\""
        echo "Launched offboard script in screen session: px4-offboard (attach with: screen -r px4-offboard)."
        return 0
    fi

    echo "No GUI terminal available; running offboard script in background."
    echo "Offboard log: $LOG_FILE"
    bash -lc "cd \"$ROOT_DIR\"; python3 \"$OFFBOARD_SCRIPT\" >> \"$LOG_FILE\" 2>&1" &
}

cd "$ROOT_DIR"

(
    sleep "$LAUNCH_DELAY_S"
    launch_offboard_terminal
) &
SLEEP_PID=$!

trap 'kill "$SLEEP_PID" 2>/dev/null || true' EXIT

make px4_sitl_default gazebo-classic
