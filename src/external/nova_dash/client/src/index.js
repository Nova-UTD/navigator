import React from "react";
import ReactDOM from "react-dom";
import Worldview from "./Worldview";
import Console from "./Console"
import SideBar from "./sidebar"

import './styles.css';

import {
  BrowserRouter as Router,
  Routes,
  Route
} from "react-router-dom";

function App() {
  return (
    <Router>
      <div style={{ width: "100vw", height: "100vh" }}>
        <SideBar />

        <Routes>
          <Route exact path="/" element={<Worldview/>}/>
          <Route exact path="/console" element={<Console/>}/>
        </Routes>
        
      </div>
    </Router>
  );
}

const rootElement = document.getElementById("root");
ReactDOM.render(<App />, rootElement);