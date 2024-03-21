<script lang="ts">
	import type { WSMessage } from '$lib/api';
	import { onMount } from 'svelte';
	import { Terminal } from 'xterm';
	import { FitAddon } from 'xterm-addon-fit';
	import 'xterm/css/xterm.css';

	const term = new Terminal({
		cursorBlink: true,
		lineHeight: 1.25
	});
	const fitAddon = new FitAddon();
	term.loadAddon(fitAddon);

	let terminalElement: HTMLDivElement | undefined;

	let ws: WebSocket | undefined;

	onMount(() => {
		ws = new WebSocket('ws://localhost:8000/term/ws');
		ws.onopen = () => {
			if (terminalElement) term.open(terminalElement);
			term.writeln(
				[
					'\x1b[32m.___.        , .  ', // prevent formating
					'  _/  _ ._ *-+-|_ ',
					'./__.(/,[ )| | [ )'
				].join('\n\r')
			);
			fitAddon.fit();
			const resizeMessage: WSMessage = {
				type: 'resize',
				cols: term.cols,
				rows: term.rows
			};
			ws?.send(JSON.stringify(resizeMessage));
		};

		ws.onmessage = (event) => {
			term.write(event.data);
		};
	});

	term.onKey((e) => {
		if (!ws) return;

		if (ws.readyState === ws.OPEN) {
			const inputMessage: WSMessage = {
				type: 'input',
				key: e.key
			};
			ws.send(JSON.stringify(inputMessage));
		}
	});
</script>

<div bind:this={terminalElement} id="terminal" class="w-full h-full"></div>
