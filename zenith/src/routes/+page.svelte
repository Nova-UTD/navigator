<script lang="ts">
	import { LaunchList, LaunchListSkeleton, LaunchCreateDialog } from '$lib/components/launch-list';
	import { LaunchEditor } from '$lib/components/launch-editor';
	import { Button } from '$lib/components/ui/button';
	import PlusSquare from 'lucide-svelte/icons/plus-square';
	import { launchStore, selectedLaunchStore, addNodes, removeNodes } from '$lib/stores/launchStore';
	import { subsystemStore } from '$lib/stores/subsystemStore';
	import { loadAll } from '@square/svelte-store';
	import { Terminal } from '$lib/components/ui/terminal';
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
				onLaunchSelect={(index) => ($launchStore.selectedLaunch = index)}
				launches={$launchStore.launches.map((launch, index) => ({
					name: launch.metadata.name,
					selected: $launchStore.selectedLaunch === index
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
	<section class="w-full h-full overflow-hidden">
		<article class="flex justify-between items-center">
			<h1>Launch List</h1>
			<Button variant="secondary">Launch</Button>
		</article>
		<Terminal launchCommand={`ros2 launch ${$selectedLaunchStore?.path ?? ''}`} />
	</section>
</section>
