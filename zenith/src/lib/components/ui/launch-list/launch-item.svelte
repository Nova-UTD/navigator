<script lang="ts">
	import { Dot } from '$lib/components/ui/dot';
	import type { LaunchItemProps } from '.';

	type $$Props = LaunchItemProps;
	export let name: $$Props['name'];
	export let selected: $$Props['selected'] = false;
	export let index: $$Props['index'];
	export let onClick: $$Props['onClick'];
</script>

<button
	class="w-full px-9 py-4 border-x border-b border-2 border-border flex items-center justify-between hover:bg-border cursor-pointer"
	on:click={() => onClick(index)}
>
	{name}
	{#if selected}
		<Dot />
	{/if}
</button>
