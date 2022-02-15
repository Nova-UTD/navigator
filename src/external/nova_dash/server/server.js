const express = require('express');
const path = require('path');
const WebSocket = require('ws');
const bodyParser = require('body-parser')

// Define express app and websocket server
const app = express();
const port = 3001;
const socketServer = new WebSocket.Server({port: 3030});

// App using json formats
app.use(bodyParser.urlencoded({extended: false})) 
app.use(bodyParser.json()) 

app.listen(port, () => { // On app open
  console.log(`Example app listening at http://localhost:${port}`)
})

const messages = []; // Message array
app.post('/console', function (req, res) { // Console log POST request
  // Get arguments from request body
  var group = req.body.group;
  var name = req.body.name;
  var priority = req.body.priority;
  var message = req.body.message;
  var time = req.body.time;

  // Get message latency
  const latency = Date.now() - time;

  // Respond with callback string
  const string = "Callback from:\n\t" + group + "/" + name + "\nPriority:\n\t" + priority + "\nMessage:\n\t" + message + "\nTime:\n\t" + time + "\nLatency:\n\t" + latency + "ms"
  res.send(string);

  // Push into message array
  messages.push(string);

  socketServer.clients.forEach((client) => { // Push to each client
    if (client.readyState === WebSocket.OPEN) { // If client is ready
      client.send(JSON.stringify([string]));
      console.log(JSON.stringify([string]));
    }
  });
})

// Dummy boxes
let box = {
  id: "010",
  name: "BOX",
  pose: {
    orientation: { x: 0, y: 0, z: 0, w: 1 },
    position: { x: -5, y: 0, z: 0 },
  },
  scale: { x: 15, y: 15, z: 15 },
    color: { r: 0, g: 1, b: 1, a: 0.8 },
}

let box2 = {
  id: "010",
  name: "BOX1",
  pose: {
    orientation: { x: 0, y: 0, z: 0, w: 1 },
    position: { x: 10, y: 0, z: 0 },
  },
  scale: { x: 15, y: 15, z: 15 },
    color: { r: 0, g: 1, b: 1, a: 0.8 },
}

let box3 = {
  id: "011",
  name: "BOX2",
  pose: {
    orientation: { x: 0, y: 0, z: 0, w: 1 },
    position: { x: -5, y: 0, z: 0 },
  },
  scale: { x: 15, y: 15, z: 15 },
    color: { r: 0, g: 1, b: 1, a: 0.8 },
}

let box4 = {
  id: "010",
  name: "BOX1",
  pose: {
    orientation: { x: 0, y: 0, z: 0, w: 1 },
    position: { x: 0, y: 20, z: 0 },
  },
  scale: { x: 15, y: 15, z: 15 },
    color: { r: 0, g: 1, b: 1, a: 0.8 },
}

let activeObjects = []; // Active object array

socketServer.on('connection', (socketClient) => { // On client connection event
  console.log('New client connected');
  console.log('Number of clients: ', socketServer.clients.size);

  let conn = setInterval(function(){ // Connection loop
    socketClient.send( JSON.stringify(activeObjects) ); // Send current active objects to client
  }, 1000); // Every 1000ms
    
  // Dummy messages for testing
  socketClient.send( JSON.stringify(box) );
  setTimeout(function(){
    activeObjects = [box2];
  }, 2000);
  setTimeout(function(){
    activeObjects = [box3];
  }, 4000);
  setTimeout(function(){
    activeObjects = [box4];
  }, 6000);

  socketClient.on('close', (socketClient) => { // On client disconnect
    clearInterval(conn); // Clear the connection loop
    console.log('Client closed');
    console.log('Number of clients: ', socketServer.clients.size);
    });
});