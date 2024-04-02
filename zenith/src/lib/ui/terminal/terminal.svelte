<script lang="ts">
	import type { WSMessage } from '$lib/api';
	import { cn } from '$lib/utils';
	import { onMount } from 'svelte';
	import type { HTMLAttributes } from 'svelte/elements';
	import { Terminal } from 'xterm';
	import { FitAddon } from 'xterm-addon-fit';
	import 'xterm/css/xterm.css';

	type $$Props = HTMLAttributes<HTMLDivElement>;
	let className: $$Props['class'] = '';
	export { className as class };

	const term = new Terminal({
		cursorBlink: true,
		lineHeight: 1.25
	});
	const fitAddon = new FitAddon();
	term.loadAddon(fitAddon);

	let terminalElement: HTMLDivElement | undefined;

	let ws: WebSocket | undefined;

	function resizeTerminal() {
		if (!terminalElement) return;
		fitAddon.fit();
		const resizeMessage: WSMessage = {
			type: 'resize',
			cols: term.cols,
			rows: term.rows
		};
		ws?.send(JSON.stringify(resizeMessage));
	}

	export function runLaunchCommand(launchCommand: string) {
		if (!ws) return;

		if (ws.readyState === ws.OPEN) {
			const launchMessage: WSMessage = {
				type: 'input',
				key: launchCommand
			};
			ws.send(JSON.stringify(launchMessage));

			const enterMessage: WSMessage = {
				type: 'input',
				key: '\r'
			};
			ws.send(JSON.stringify(enterMessage));
		}
	}

	onMount(() => {
		// TODO: Change to dynmaic URL.
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
			resizeTerminal();
			window.addEventListener('resize', resizeTerminal);
		};

		ws.onmessage = (event) => {
			term.write(event.data);
		};

		return () => {
			ws?.close();
			window.removeEventListener('resize', resizeTerminal);
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

<div bind:this={terminalElement} id="terminal" class={cn('w-full h-full', className)}></div>
