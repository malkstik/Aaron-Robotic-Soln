CONTAINER_NAME="machinalabs"
DOCKER_OPTIONS=""

DOCKER_OPTIONS+="--ssh default "
DOCKER_OPTIONS+="-t $CONTAINER_NAME "
DOCKERFILE_DIR=$1

DOCKER_BUILDKIT=1 docker build $DOCKER_OPTIONS $DOCKERFILE_DIR