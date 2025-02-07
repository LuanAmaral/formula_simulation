#!/bin/bash

IMAGE_NAME="formula-simulation"

RED='\033[0;31m'
GREEN='\033[0;32m'
NC='\033[0m' # No Color

xhost +local:root

echo $1

# Check for a "rebuild" flag
if [[ "$1" == "--rebuild" ]]; then
  echo -e "${GREEN}Rebuilding image $IMAGE_NAME${NC}"
  docker build -t $IMAGE_NAME .
else
  if docker images | grep -q $IMAGE_NAME; then
    #write in green
    echo -e "${GREEN}Image $IMAGE_NAME already exists${NC}"
  else
    #write in red
    echo -e "${RED}Image $IMAGE_NAME does not exist${NC}"
    docker build -t $IMAGE_NAME .
  fi
fi
# docker build -t formula-simulation .

docker compose up -d

docker exec -it formula_simulation bash

docker compose down

xhost -local:root