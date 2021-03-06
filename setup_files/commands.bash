export HOME="/home/ubuntu"

function start_pineapple() {
    echo "Starting"
    screen -S controller -d -m "$HOME/code/pineapple/setup_files/start_pineapple.bash"
    sleep 3
    echo "$(screen -ls)"
}

function compile_and_start_pineapple() {
    stop_pineapple
    cd $HOME/code/catkin_ws/
    catkin_make install
    sleep 5
    start_pineapple
    cd $HOME/code/pineapple
}

function stop_pineapple() {
    /home/ubuntu/code/pineapple/setup_files/stop_pineapple.bash
}

