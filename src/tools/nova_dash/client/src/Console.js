import React, { useState } from "react";
import { w3cwebsocket as W3CWebSocket } from "websocket";

// New websocket client
const client = new W3CWebSocket('ws://localhost:3030');

export default function Example() {
    let messages = [];

    const getElement = (id) => document.getElementById(id);

    // Add message when string inputted
    const addMessage = (message) => {
        const pTag = document.createElement('p'); // Create paragraph element
        pTag.appendChild(document.createTextNode("| " + message)); // Create new text node with message
        getElement('messages').appendChild(pTag); // Find message element and put new tag in holder
    };

    client.onopen = () => { // When client establishes connection to server
        console.log('Socket Client Connected');
    };
    
    client.onmessage = (message) => { // On client message
        const data = JSON.parse(message.data); // Parse json event for data (if multiple messages)
        if (data[0] === "Message") {
            const messages = data[1]; // Parse json event for data (if multiple messages)
            messages.forEach(addMessage); // Create message out of non-json objects from messages array
        }    
    };

    return(
        <>
            <h1 style={{textAlign: "center"}}>Nova | Dash</h1>

            <div id="messages"></div>
        </>
    );
}