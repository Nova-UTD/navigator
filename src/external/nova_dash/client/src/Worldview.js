import React, { useState } from "react";
import Worldview, { Grid, Text, Cubes, Lines, Axes } from "regl-worldview";
import { w3cwebsocket as W3CWebSocket } from "websocket";

// New websocket client
const client = new W3CWebSocket('ws://localhost:3030');

function returnObjects(message) { // Creates objects from converted message.data entry
  let cube = { // Cube object
    id: message.id,
    depth: {
      enable: true,
      mask: true,
    },
    blend: {
      enable: true,
      func: {
        srcRGB: "src alpha",
        srcAlpha: 1,
        dstRGB: "one minus src alpha",
        dstAlpha: 1,
      },
    },
    pose: {
      orientation: { 
        x: message.pose.orientation.x,
        y: message.pose.orientation.y,
        z: message.pose.orientation.z,
        w: message.pose.orientation.w },
      position: {
        x: message.pose.position.x,
        y: message.pose.position.y,
        z: message.pose.position.z
      },
    },
    scale: {
      x: message.scale.x,
      y: message.scale.y,
      z: message.scale.z
    },
    color: {
      r: message.color.r,
      g: message.color.g,
      b: message.color.b,
      a: message.color.a
    },
  }

  let text = { // Text object
    id: message.id,
    name: message.name,
    text: "[" + message.id + "] " + message.name,
    color: { r: 1, g: 1, b: 1, a: 1 },
    pose: {
      orientation: { 
        x: message.pose.orientation.x,
        y: message.pose.orientation.y,
        z: message.pose.orientation.z,
        w: message.pose.orientation.w },
      position: {
        x: message.pose.position.x,
        y: message.pose.position.y,
        z: message.pose.position.z
      },
    },
    scale: {
      x: message.scale.x,
      y: message.scale.y,
      z: message.scale.z
    },
  }
  
  // Bounding boxes initial positions
  let scale = message.scale;
  const p0 = [-scale.x / 2, -scale.y / 2, -scale.z / 2];
  const p1 = [scale.x / 2, -scale.y / 2, -scale.z / 2];
  const p2 = [scale.x / 2, scale.y / 2, -scale.z / 2];
  const p3 = [-scale.x / 2, scale.y / 2, -scale.z / 2];
  const p4 = [-scale.x / 2, -scale.y / 2, scale.z / 2];
  const p5 = [scale.x / 2, -scale.y / 2, scale.z / 2];
  const p6 = [scale.x / 2, scale.y / 2, scale.z / 2];
  const p7 = [-scale.x / 2, scale.y / 2, scale.z / 2];

  let lines = { // Bounding line object
    pose: message.pose,
    primitive: "lines",
    scale: { x: 0.2, y: 0.2, z: 0.2 },
    points: [
      // bottom
      p0,
      p1,
      p1,
      p2,
      p2,
      p3,
      p3,
      p0,
      // top
      p4,
      p5,
      p5,
      p6,
      p6,
      p7,
      p7,
      p4,
      // around
      p0,
      p4,
      p1,
      p5,
      p2,
      p6,
      p3,
      p7,
    ],
    color: { r: 1, g: 0, b: 0, a: 1 },
  };

  return [cube,text,lines];
}

export default function Example() {
  // Marker and text tables
  let markers = [];
  let texts = [];
  let lines = [];

  function createCube(message){
    // Check if object already exists
    
    let found = false;
    let foundKey = 0;
    // Disable error for unused value next line
    // eslint-disable-next-line
    
    // Check if object already exists
    for (const [key, value] of Object.entries(markers)) {
      if(markers[key].id === message.id) {
        found = true;
        foundKey = key;
      }
    }
    
    // Create object with box, text and line
    let objs = returnObjects(message);
    let cube = objs[0];
    let text = objs[1];
    let line = objs[2];
    
    if(found === true) { // If object already exists, replace current object with new object
      markers[foundKey] = cube;
      texts[foundKey] = text;
      lines[foundKey] = line;

    }else{ // Otherwise add the new object to the table
      markers.push(cube);
      texts.push(text);
      lines.push(line);
    }
  }
  
  client.onopen = () => { // When client establishes connection to server
    console.log('Socket Client Connected');
  };

  client.onmessage = (message) => { // On client message
    const data = JSON.parse(message.data);
    if(data[0] === "Object") {
      const newData = data[1];
      for (var i = 0; i < newData.length; i++) {
        console.log( newData[i] );
        createCube( newData[i] );
      }
    }
  };
  
  return (
    <Worldview>
      <Grid count={30} />
      <Text autoBackgroundColor>{texts}</Text>
      <Cubes>{markers}</Cubes>
      <Lines>{lines}</Lines>
      
      <Axes />
    </Worldview>
  );
}
