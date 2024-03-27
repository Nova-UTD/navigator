<script lang="ts">
	import LaunchList from './launch-list.svelte';
	import LaunchCreateDialog from './launch-create-dialog.svelte';
	import LaunchListSkeleton from './launch-list-skeleton.svelte';
	import PlusSquare from 'lucide-svelte/icons/plus-square';

	import { Button } from '$lib/ui/button';
	import { launchStore, setSelectedLaunchIndex } from '$lib/stores/launchStore';
</script>

<section class="w-full h-full flex flex-col gap-5">
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
	{:catch err}
		<p>Error: {err.message}</p>
	{/await}
</section>
