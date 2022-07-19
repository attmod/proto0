# attmod proto0 makefile
#

LOCAL_USER_ID := $(shell id -u)
export LOCAL_USER_ID

include .env

DOCKER_IMAGE := ${PROJECT_NAME}:latest
export DOCKER_IMAGE

default: build run

build:
	docker build -t ${DOCKER_IMAGE} \
			--build-arg DOCKER_SRC=${DOCKER_SRC} \
			--build-arg LOCAL_USER_ID=${LOCAL_USER_ID} .

run:
	docker-compose up --remove-orphans

stop:
	docker-compose down