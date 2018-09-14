FROM cartographer-google-base

RUN apt-get update && apt-get install -y vim wget

# Set the default directory
WORKDIR /

# Copy entrypoint file into the container at /
ADD entrypoint.sh /

# Configure environment
CMD ["/bin/bash", "entrypoint.sh"]
