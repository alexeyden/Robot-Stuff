#!/bin/sh

build() {
	docker build -f Dockerfile -t couchdb .
}

run() {
	count=$1; [[ $1 ]] || count=1

	for x in {1..$1}; do 
		docker run -u root -it /usr/bin/couchdb
	done
}

debug() {
	docker run -u root -it couchdb /bin/bash
	exit 0
}

clean() {
	echo "Cleaning dead docker containers"
	docker ps -aq | xargs docker rm
}

$1 $2 || (echo "no such command: $1" && exit 1)

