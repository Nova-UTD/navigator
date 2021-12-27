/*
 * Package:   nova_dash
 * Filename:  server.js
 * Author:    Raghav Pillai
 * Email:     raghavpillai101@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 *
 * This is the local server handler that handles the bridging and acts as a handler between core vehicle logged data and interface
*/

// Standard requirements
const express = require('express');
const path = require('path');
const WebSocket = require('ws');
const bodyParser = require('body-parser');

// Misc. declarations
const app = express(); // Express app creation
const port = 3000; // GET/POST request port
const socketServer = new WebSocket.Server({port: 3030}); // Websocket server and port

const messages = []; // Table for session messages [memory]

// Ecxpress server declarations
app.use(bodyParser.urlencoded({extended: false}))
app.use(bodyParser.json()) // for json messages
app.use("/static", express.static('./static/')); // use static for data

app.get('/', (req, res) => { // Default GET requests return html file and directory
  res.sendFile(path.join(__dirname, 'index.html'));
});

app.listen(port, () => { // Initial listening and prompt on specified port
  console.log(`Nova Server Opened`);
  console.log(`Handler listening on http://localhost:${port}`);
})

app.post('/', function (req, res) { // Default POST (for message)
  // Lazy so copy data into local variables
  var group = req.body.group;
  var name = req.body.name;
  var priority = req.body.priority;
  var message = req.body.message;
  var time = req.body.time;

  const latency = Date.now() - time; // Find latency by difference checking

  const string = "Callback from:\t" + group + "/" + name + "\nPriority:\t" + priority + "\nMessage:\t" + message + "\nTime:\t" + time + "\nLatency:\t" + latency + "ms" + "\n";
  res.send(string); // Respond with confirmation ticket
  console.log(string); // Print to console

  messages.push(string); // Store message in array for new clients

  socketServer.clients.forEach((client) => { // Push to each client
    if (client.readyState === WebSocket.OPEN) { // If client exists [ws.open]
      client.send(JSON.stringify([string])); // put into json for client.js
      console.log("Served client message");
      //console.log(JSON.stringify([string]));
    }
  });
})

socketServer.on('connection', (socketClient) => { // New connection [socketclient = client connection]
  console.log('Client Connected'); // New client log
  console.log('Number of clients: ', socketServer.clients.size); // Reprint number of total clients
  socketClient.send(JSON.stringify(messages)); // Send initial messages to client

  socketClient.on('close', (socketClient) => { // On client close
    console.log('Client Disconnected'); // Console log as closed
    console.log('Number of clients: ', socketServer.clients.size); // Reprint number of total clients
  });
});