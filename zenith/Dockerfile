FROM navigator:latest 

RUN mkdir -p /navigator/zenith

# Set working directory to /app.
WORKDIR /navigator/zenith

# Add Node installed binaries to $PATH.
ENV PATH /navigator/zenith/node_modules/.bin:$PATH

# Install Node.js.
RUN curl -fsSL https://deb.nodesource.com/setup_20.x | bash - && \
    apt-get install -y nodejs

# NPM install.
COPY package.json ./
COPY package-lock.json ./
RUN npm install

# Install PyInstaller for Python shim.
RUN pip install -U pyinstaller

ENTRYPOINT [ "bash" ]