<script lang="ts">
	import LaunchEditor from './launch-editor.svelte';
	import { loadAll } from '@square/svelte-store';
	import { subsystemStore } from '$lib/stores/subsystemStore';
	import { launchStore, selectedLaunchStore } from '$lib/stores/launchStore';
	import { addNodes, removeNodes } from '$lib/stores/launchStore';
</script>

<div class="w-full h-full overflow-hidden">
	<h1>Launch Editor</h1>
	{#await loadAll([launchStore, subsystemStore])}
		<p>Loading...</p>
	{:then}
		{#if typeof $selectedLaunchStore != 'undefined'}
			<LaunchEditor
				onNodeChanged={(node, checked) => {
					if (checked) {
						addNodes([node]);
					} else {
						removeNodes([node]);
					}
				}}
				subsystems={$subsystemStore}
				activeNodes={$selectedLaunchStore.nodes}
			/>
		{:else}
			<p>
				It seems that the launch editor couldn't load. Make sure you create a launch file first.
			</p>
		{/if}
	{:catch err}
		Error: {err.message}
	{/await}
</div>
