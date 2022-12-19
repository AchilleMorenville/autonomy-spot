FROM ubuntu/nginx:1.18-20.04_beta
 
#RUN mkdir /etc/nginx/sites-available
#RUN mkdir /etc/nginx/sites-enabled

COPY server.conf /etc/nginx/sites-available/default

#RUN ln -s /etc/nginx/sites-available/server /etc/nginx/sites-enabled/default

RUN apt update 
RUN apt install -y bash sudo curl iputils-ping net-tools wget nano

WORKDIR /src/

RUN openssl req -x509 -nodes -days 365 -subj  "/C=CA/ST=QC/O=BUILDWISE/CN=localhost"  -newkey rsa:2048 -keyout  localhost.key -out localhost.crt

RUN wget  https://deb.nodesource.com/setup_14.x 
RUN sh /src/setup_14.x

RUN apt update
RUN apt install nodejs

#WORKDIR /src/
#RUN wget https://github.com/potree/potree/releases/download/1.8/Potree_1.8.zip
#RUN unzip Potree_1.8.zip
#WORKDIR /src/Potree_1.8

# WORKDIR /src/
# COPY ./frontend /src/frontend/
# WORKDIR /src/frontend
#RUN npm install
#RUN echo 'service nginx restart' >> /bootstrap.sh
#RUN  echo 'npm start' >> /bootstrap.sh
#RUN chmod +x /bootstrap.sh
#CMD /bootstrap.sh
