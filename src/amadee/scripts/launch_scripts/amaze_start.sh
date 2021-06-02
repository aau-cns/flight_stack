#!/bin/sh

PI1_IP=192.168.0.105
PI2_IP=192.168.0.195

# connect to ssh
# https://superuser.com/questions/8673/how-can-i-use-ssh-to-run-a-command-on-a-remote-unix-machine-and-exit-before-the
#ssh amadee1_w_core -f 'roscore'
#ssh core@${PI1_IP} -f 'bash -c "roscore"'

# launch pi1 interactively
# https://unix.stackexchange.com/questions/243821/run-a-command-in-an-interactive-shell-with-ssh-after-sourcing-bashrc
#ssh amadee1_w_core -f 'bash -ic "roslaunch amadee_bringup amaze.launch pi:=1"'
#ssh core@${PI1_IP} -f 'bash -ic "roslaunch amadee_bringup amaze.launch pi:=1"'
# NOT WORKING, stuff cannot be killed

# ssh -f -N -M -S <path-to-socket> -L <port>:<host>:<port> <server>
# ssh -f -t -N -M -S /tmp/session_test core@${PI1_IP} 'bash -ic "roslaunch amadee_bringup amadee_operator.launch pi:=1"'
#ssh -t  core@${PI1_IP} 'bash -ic "roslaunch amadee_bringup amadee_operator.launch pi:=1"'

# ssh -t core@${PI2_IP} 'bash -c "roslaunch amadee_bringup amaze.launch pi:=2"'
#
# gnome-terminal --tab
# ssh -t core@${PI1_IP} 'bash -c "/home/core/start_pi.sh"'

# ==============================================
# OR

CMD1="ssh -t core@${PI1_IP} 'bash -c \"/home/core/amadee_pi1.sh\"'"
CMD2="ssh -t core@${PI2_IP} 'bash -c \"/home/core/amadee_pi2.sh\"'"

# Create Tmux Session
CUR_DATE=`date +%F-%H-%M-%S`
export SES_NAME="amaze_${CUR_DATE}"

## INFO(martin): use .0 .1 if base index has not been configured
tmux new -d -s "${SES_NAME}"

# ROSCORE
tmux rename-window 'pi1'
tmux send-keys -t ${SES_NAME}.1 "${CMD1}" 'C-m'

# BACKGROUND
tmux new-window -n 'pi2'
tmux send-keys -t ${SES_NAME}.1 "${CMD2}" 'C-m'

tmux select-window -t 'pi1'
tmux attach -t ${SES_NAME}
