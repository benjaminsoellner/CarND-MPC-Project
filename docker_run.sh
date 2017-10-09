docker-machine start default
docker-machine env default
export SCRIPT=$(readlink -f "$0")
export SCRIPTPATH=$(dirname "$SCRIPT")
export HOST_VIRTUAL_IP=192.168.99.1
export DISPLAY=127.0.0.1:0
# before, run:
# docker build . -t carndmpcquizzes 
docker run --name carndmpcproject-0 --rm --privileged -dt \
  --memory=8g \
  -p 2223:22 \
  -v $SCRIPTPATH:/src \
  -e DISPLAY \
  carndmpcproject
