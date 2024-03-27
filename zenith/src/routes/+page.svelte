<script lang="ts">
	import { LaunchEditor } from '$lib/modules/launch-editor';
	import { launchStore, selectedLaunchStore, addNodes, removeNodes } from '$lib/stores/launchStore';
	import { subsystemStore } from '$lib/stores/subsystemStore';
	import { loadAll } from '@square/svelte-store';
	import { LaunchTerminal } from '$lib/modules/launch-terminal';
	import { LaunchListPanel } from '$lib/modules/launch-list';
</script>

<section class="w-full h-full overflow-hidden grid grid-cols-[0.5fr_1fr_1fr] gap-[5%]">
	<LaunchListPanel />
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
	<LaunchTerminal />
</section>
