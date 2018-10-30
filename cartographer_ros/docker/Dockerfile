FROM robopaas/cartographer_base_image:melodic

RUN apt-get update && apt-get install -y wget

# Set the default directory
WORKDIR /

# Copy entrypoint file into the container at /
ADD entrypoint.sh /

# Configure environment
CMD ["/bin/bash", "entrypoint.sh"]
