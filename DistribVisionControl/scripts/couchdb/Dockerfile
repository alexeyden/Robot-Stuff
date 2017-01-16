FROM ubuntu 

RUN apt-get update
RUN apt-get install vim -y
RUN apt-get install software-properties-common -y

RUN add-apt-repository ppa:couchdb/stable -y
RUN apt-get update
RUN apt-get install couchdb -y

RUN mkdir /var/run/couchdb
ADD default.ini /etc/couchdb/default.ini

