.PHONY: interview-base-ubuntu18.04-cudagl-10.2

all: interview-base-ubuntu18.04-cudagl-10.2


interview-base-ubuntu18.04-cudagl-10.2:
	docker build -t tii/interview-base-ubuntu18.04-cudagl-10.2 . -f Dockerfile_interview-base-ubuntu18.04-cudagl-10.2


