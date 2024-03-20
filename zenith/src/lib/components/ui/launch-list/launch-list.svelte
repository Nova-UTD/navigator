<script lang="ts">
	import LaunchItem from '$lib/components/ui/launch-list/launch-item.svelte';
	import type { LaunchListProps } from '.';

	type $$Props = LaunchListProps;
	export let launches: $$Props['launches'];
	export let onLaunchSelect: $$Props['onLaunchSelect'];
</script>

<ul class="flex flex-col gap-y-2">
	{#each launches as { name, selected }, i}
		<li>
			<LaunchItem index={i} onClick={onLaunchSelect} {name} {selected} />
		</li>
	{/each}
</ul>
