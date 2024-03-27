<script lang="ts">
	import LaunchItem from './launch-item.svelte';
	import LaunchItemMenu from './launch-item-menu.svelte';
	import type { LaunchListProps } from '.';

	type $$Props = LaunchListProps;
	export let launches: $$Props['launches'];
	export let onLaunchSelect: $$Props['onLaunchSelect'];
</script>

<ul class="flex flex-col gap-y-2">
	{#each launches as { name, path, selected }, i}
		<li>
			<LaunchItemMenu {name} {path}>
				<LaunchItem index={i} onClick={onLaunchSelect} {name} {selected} />
			</LaunchItemMenu>
		</li>
	{/each}
</ul>
