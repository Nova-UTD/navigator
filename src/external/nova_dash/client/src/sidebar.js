import React from 'react';
import { slide as Menu } from 'react-burger-menu';

export default props => {
  return (
    <Menu>
      <a className="menu-item" href="/">
        3D View
      </a>

      <a className="menu-item" href="/URL">
        Console View
      </a>
    </Menu>
  );
};