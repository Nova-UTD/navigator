const getElement = (id) => document.getElementById(id);
const addMessage = (message) => {
    const pTag = document.createElement('p');
    pTag.appendChild(document.createTextNode("| " + message));
    getElement('messages').appendChild(pTag);
};
            
const ws = new WebSocket('ws://localhost:3030');
ws.onopen = () => { 
    console.log('Now connected'); 
};
            
ws.onmessage = (event) => {
    const messages = JSON.parse(event.data);
    messages.forEach(addMessage);
};