const express = require('express');
const path = require('path');
const WebSocket = require('ws');
const bodyParser = require('body-parser')

const app = express();
const port = 3000;
const socketServer = new WebSocket.Server({port: 3030});

const messages = [];

app.use(bodyParser.urlencoded({extended: false})) 
app.use(bodyParser.json()) 
app.use("/static", express.static('./static/'));

app.get('/', (req, res) => {
  res.sendFile(path.join(__dirname, 'index.html'));
});

app.listen(port, () => {
    console.log(`Example app listening at http://localhost:${port}`)
})

app.post('/', function (req, res) {
    var group = req.body.group;
    var name = req.body.name;
    var priority = req.body.priority;
    var message = req.body.message;
    var time = req.body.time;

    const latency = Date.now() - time;

    const string = "Callback from:\n\t" + group + "/" + name + "\nPriority:\n\t" + priority + "\nMessage:\n\t" + message + "\nTime:\n\t" + time + "\nLatency:\n\t" + latency + "ms"
    res.send(string);

    messages.push(string);

    socketServer.clients.forEach((client) => { // Push to each client
        if (client.readyState === WebSocket.OPEN) {
          client.send(JSON.stringify([string]));
          console.log(JSON.stringify([string]));
        }
    });
})

socketServer.on('connection', (socketClient) => {
    console.log('connected');
    console.log('Number of clients: ', socketServer.clients.size);
    socketClient.send(JSON.stringify(messages)); // Send initial messages to client

    socketClient.on('close', (socketClient) => {
      console.log('closed');
      console.log('Number of clients: ', socketServer.clients.size);
    });
});