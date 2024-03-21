export type InputMessage = {
	type: 'input';
	key: string;
};

export type ResizeMessage = {
	type: 'resize';
	cols: number;
	rows: number;
};

export type WSMessage = InputMessage | ResizeMessage;
