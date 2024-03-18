<script lang="ts">
	import LaunchItem from '$lib/components/ui/launch-list/launch-item.svelte';
	import type { LaunchListProps } from '.';

	type $$Props = LaunchListProps;
	export let launches: $$Props['launches'];
	export let onLaunchSelect: $$Props['onLaunchSelect'];
</script>

<ul>
	<li class="">
		{#each launches as { name, selected }, i}
			<LaunchItem index={i} onClick={onLaunchSelect} {name} {selected} />
		{/each}
	</li>
</ul>
