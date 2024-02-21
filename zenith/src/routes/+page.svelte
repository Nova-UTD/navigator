<script>
	import { launchList, subsystemList } from '$lib/cli-bindings';
	import { LaunchList } from '$lib/components/ui/launch-list';
	import { LaunchEditor } from '$lib/components/ui/launch-editor';

	let launchListPromise = launchList('../../launches');
	let subsystemPromise = subsystemList();
	let selectedIndex = 0;
</script>

<section class="w-full h-full overflow-hidden grid grid-cols-[1fr_2fr_1fr] gap-[5%]">
	{#await launchListPromise then launchList}
		<LaunchList
			launches={launchList.map((launch, index) => ({
				name: launch.metadata.name,
				selected: selectedIndex === index
			}))}
		/>
		{#await subsystemPromise then subsystemsResponse}
			<LaunchEditor subsystems={subsystemsResponse.subsystems} />
		{:catch err}
			Error: {err.message}
		{/await}
	{:catch err}
		Error: {err.message}
	{/await}
</section>
