<script lang="ts">
	import { LaunchList, LaunchListSkeleton } from '$lib/components/ui/launch-list';
	import { LaunchEditor } from '$lib/components/ui/launch-editor';
	import { Button } from '$lib/components/ui/button';
	import PlusSquare from 'lucide-svelte/icons/plus-square';
	import { launchStore, selectedLaunchStore, addNodes, removeNodes } from '$lib/stores/launchStore';
	import { subsystemStore } from '$lib/stores/subsystemStore';
	import { loadAll } from '@square/svelte-store';
	import LaunchCreateDialog from '$lib/components/ui/launch-list/launch-create-dialog.svelte';
</script>

<section class="w-full h-full overflow-hidden grid grid-cols-[1fr_2fr_1fr] gap-[5%]">
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
				onLaunchSelect={(index) => ($launchStore.selectedLaunch = index)}
				launches={$launchStore.launches.map((launch, index) => ({
					name: launch.metadata.name,
					selected: $launchStore.selectedLaunch === index
				}))}
			/>
		{/await}
	</section>
	{#await loadAll([launchStore, subsystemStore]) then}
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
</section>
