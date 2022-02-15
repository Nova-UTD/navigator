import React from "react";
import ReactDOM from "react-dom";
import Example from "./Example";

function App() {
  return (
    <div style={{ width: "100vw", height: "100vh" }}>
      <Example />
    </div>
  );
}

const rootElement = document.getElementById("root");
ReactDOM.render(<App />, rootElement);