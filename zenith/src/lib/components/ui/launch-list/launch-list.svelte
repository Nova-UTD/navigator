<script lang="ts">
	import LaunchItem from '$lib/components/ui/launch-list/launch-item.svelte';
	import type { LaunchListProps } from '.';

	type $$Props = LaunchListProps;
	export let launches: $$Props['launches'];
</script>

<ul>
	<li class="">
		{#each launches as { name, selected }}
			<LaunchItem {name} {selected} />
		{/each}
	</li>
</ul>
