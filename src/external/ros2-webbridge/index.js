// Nova server (ROS2 Node)

'use strict';

// dependencies
const rclnodejs = require('rclnodejs');
const WebSocket = require('ws');
const Bridge = require('./lib/bridge.js');
const debug = require('debug')('ros2-web-bridge:index');

// rclnodejs node
let node;

// websocket server (or client if client mode set via --address)
let server;
let connectionAttempts = 0;

// Map of bridge IDs to Bridge objects
let bridgeMap = new Map();

function closeAllBridges() { // Close all bridge objects
  bridgeMap.forEach((bridge, bridgeId) => {
    bridge.close();
  });
}

function shutDown(error) { // Properly shutdown server functions
  // Closing the server triggers the individual connections to be closed.
  if (server) {
    server.close();
  }
  if (!rclnodejs.isShutdown()) { // don't double shutdown
    rclnodejs.shutdown();
  }
  if (error) {
    throw error;
  }
}

function createServer(options) { // Create server (from startup)
  options = options || {};
  options.address = options.address || null;
  process.on('exit', () => {
    debug('Application will exit.');
    shutDown();
  });
  return rclnodejs.init()
    .then(() => {
      node = rclnodejs.createNode('ros2_web_bridge');
      debug('ROS2 node started');
      let timeout = options.delay_between_messages;
      if (timeout == undefined) {
        timeout = 0;
      }
      createConnection(options);
      spin(node, timeout);
    })
    .catch(error => shutDown(error));
}

function spin(node, timeout) {
  if (rclnodejs.isShutdown()) {
    shutDown();
    return;
  }
  node.spinOnce();
  setTimeout(spin, timeout, node, timeout);
}

function createConnection(options) { // Create connection function (bridge + client)
  if (options.address != null) { // If no address then do default port of 9090
    debug('Starting in client mode; connecting to ' + options.address);
    server = new WebSocket(options.address);
  } else {
    options.port = options.port || 9090;
    debug('Starting server on port ' + options.port);
    server = new WebSocket.Server({port: options.port});
  }

  const makeBridge = (ws) => { // New bridge
    let bridge = new Bridge(node, ws, options.status_level);
    bridgeMap.set(bridge.bridgeId, bridge);

    bridge.on('error', (error) => { // remove bridge if erroring
      debug(`Bridge ${bridge.bridgeId} closing with error: ${error}`);
      bridge.close();
      bridgeMap.delete(bridge.bridgeId);
    });

    bridge.on('close', (bridgeId) => { // when bridge closed
      bridgeMap.delete(bridgeId);
    });
  };

  server.on('open', () => { // On bridge connection with webhook
    debug('Connected as client');
    connectionAttempts = 0;
  });
  
  if (options.address) { // Creation with port
    makeBridge(server);
  } else {
    server.on('connection', makeBridge);
  }

  server.on('error', (error) => { // Close server on error automatically
    closeAllBridges();
    debug(`WebSocket error: ${error}`);
  });

  server.on('close', (event) => { // if bridge closing try reconnect
    debug(`Websocket closed: ${event}`);
    if (options.address) {
      closeAllBridges();
      connectionAttempts++;
      // Gradually increase reconnection interval to prevent overloading server
      const delay = Math.pow(1.5, Math.min(10, Math.floor(Math.random() * connectionAttempts)));
      debug(`Reconnecting to ${options.address} in ${delay.toFixed(2)} seconds`);
      setTimeout(() => createConnection(options), delay*1000);
    }
  });

  let wsAddr = options.address || `ws://localhost:${options.port}`;
  console.log(`Nova websocket started on ${wsAddr}`);
  // gracefuly shutdown rosbridge node commanding terminal
  process.on('SIGINT', () => process.exit(1));
}

module.exports = {
  createServer: createServer,
};
