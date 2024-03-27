<script lang="ts">
	import { LaunchList, LaunchListSkeleton, LaunchCreateDialog } from '$lib/modules/launch-list';
	import { LaunchEditor } from '$lib/modules/launch-editor';
	import { Button } from '$lib/ui/button';
	import PlusSquare from 'lucide-svelte/icons/plus-square';
	import {
		launchStore,
		selectedLaunchStore,
		addNodes,
		removeNodes,
		setSelectedLaunchIndex
	} from '$lib/stores/launchStore';
	import { subsystemStore } from '$lib/stores/subsystemStore';
	import { loadAll } from '@square/svelte-store';
	import { Terminal } from '$lib/ui/terminal';
	import { LaunchTerminal } from '$lib/modules/launch-terminal';
</script>

<section class="w-full h-full overflow-hidden grid grid-cols-[0.5fr_1fr_1fr] gap-[5%]">
	<section class="flex flex-col gap-5">
		<article class="flex justify-between items-center">
			<h1>Launch List</h1>
			<LaunchCreateDialog>
				<Button variant="ghost"><PlusSquare /></Button>
			</LaunchCreateDialog>
		</article>
		{#await launchStore.load()}
			<LaunchListSkeleton />
			<p>Loading...</p>
		{:then}
			<LaunchList
				onLaunchSelect={setSelectedLaunchIndex}
				launches={$launchStore.launches.map((launch, index) => ({
					name: launch.metadata.name,
					path: launch.path,
					selected: $launchStore.selectedLaunchIndex === index
				}))}
			/>
		{/await}
	</section>
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
		{/if}
	{:catch err}
		Error: {err.message}
	{/await}
	<LaunchTerminal />
</section>
