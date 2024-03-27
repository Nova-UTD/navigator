<script lang="ts">
	import LaunchItem from './launch-item.svelte';
	import LaunchItemMenu from './launch-item-menu.svelte';
	import type { LaunchListProps } from '.';

	type $$Props = LaunchListProps;
	export let launches: $$Props['launches'];
	export let onLaunchSelect: $$Props['onLaunchSelect'];
</script>

<ul class="flex flex-col gap-y-2">
	{#each launches as { name, selected }, i}
		<li>
			<LaunchItemMenu>
				<LaunchItem index={i} onClick={onLaunchSelect} {name} {selected} />
			</LaunchItemMenu>
		</li>
	{/each}
</ul>
