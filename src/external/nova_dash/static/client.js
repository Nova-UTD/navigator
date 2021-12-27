/*
 * Package:   nova_dash
 * Filename:  client.js
 * Author:    Raghav Pillai
 * Email:     raghavpillai101@gmail.com
 * Copyright: 2021, Nova UTD
 * License:   MIT License
 * 
 * This is a script attached to index.html that handles client-server interaction and attaches onto the handler websocket
*/


// Gets all element of certain ID
const getElement = (id) => document.getElementById(id);

// Add message when string inputted
const addMessage = (message) => {
    const pTag = document.createElement('p'); // Create paragraph element
    pTag.appendChild(document.createTextNode("| " + message)); // Create new text node with message
    getElement('messages').appendChild(pTag); // Find message element and put new tag in holder
};
            
const ws = new WebSocket('ws://localhost:3030'); // Establish connection with websocket

// Function on websocket open
ws.onopen = () => {  
    console.log('Now connected'); // For debugging
};

// On websocket event (message adds)
ws.onmessage = (event) => { 
    const messages = JSON.parse(event.data); // Parse json event for data (if multiple messages)
    messages.forEach(addMessage); // Create message out of non-json objects from messages array
};