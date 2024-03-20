<script lang="ts">
	import {
		getLaunchList,
		updateLaunchList,
		subsystemList,
		type Node,
		type SubsystemInfo
	} from '$lib/api';
	import { LaunchList, LaunchListSkeleton } from '$lib/components/ui/launch-list';
	import { LaunchEditor } from '$lib/components/ui/launch-editor';
	import { isNodeEqual } from '$lib/utils';

	let launchListPromise = getLaunchList('../launches');
	let subsystemPromise = subsystemList();
	let selectedIndex = 0;

	let launchEditorState: Promise<{
		subsystems: SubsystemInfo[];
		activeNodes: Node[];
		launchPath: string;
	}>;

	$: launchEditorState = Promise.allSettled([
		subsystemPromise,
		launchListPromise.then((launchList) => launchList[selectedIndex].nodes),
		launchListPromise.then((launchList) => launchList[selectedIndex].path)
	]).then(([subsystemsResponse, activeNodesResponse, path]) => ({
		subsystems: subsystemsResponse.status === 'fulfilled' ? subsystemsResponse.value : [],
		activeNodes: activeNodesResponse.status === 'fulfilled' ? activeNodesResponse.value : [],
		launchPath: path.status === 'fulfilled' ? path.value : ''
	}));
</script>

<section class="w-full h-full overflow-hidden grid grid-cols-[1fr_2fr_1fr] gap-[5%]">
	<article class="flex flex-col gap-5">
		<h1>Launch List</h1>
		{#await launchListPromise}
			<LaunchListSkeleton />
		{:then launchList}
			<LaunchList
				onLaunchSelect={(index) => (selectedIndex = index)}
				launches={launchList.map((launch, index) => ({
					name: launch.metadata.name,
					selected: selectedIndex === index
				}))}
			/>
		{:catch err}
			Error: {err.message}
		{/await}
	</article>
	{#await launchEditorState then launchEditor}
		<LaunchEditor
			onNodeChanged={(node, checked) => {
				if (checked) {
					updateLaunchList({
						add_nodes: [node],
						path: launchEditor.launchPath
					});
				} else {
					updateLaunchList({
						remove_nodes: [node],
						path: launchEditor.launchPath
					});
				}
			}}
			subsystems={launchEditor.subsystems}
			activeNodes={launchEditor.activeNodes}
		/>
	{:catch err}
		Error: {err.message}
	{/await}
</section>
