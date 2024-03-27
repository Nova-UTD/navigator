<script lang="ts">
	import { Button } from '$lib/ui/button';
	import { Terminal } from '$lib/ui/terminal';
	import { selectedLaunchStore } from '$lib/stores/launchStore';
	import Loader2 from 'lucide-svelte/icons/loader-2';

	let terminal: Terminal;
</script>

<section class="w-full h-full overflow-hidden flex flex-col">
	<article class="flex justify-between items-center">
		<h1>Launch List</h1>
		{#await selectedLaunchStore.load()}
			<Button disabled variant="secondary" class="flex items-center justify-center">
				<Loader2 class="w-4 h-4 animate-spin" />
			</Button>
		{:then selectedLaunch}
			<Button
				on:click={() => terminal.runLaunchCommand(`ros2 launch ${selectedLaunch?.path ?? ''}`)}
				variant="secondary">Launch</Button
			>
		{/await}
	</article>

	<Terminal bind:this={terminal} />
</section>
