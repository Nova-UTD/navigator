<script lang="ts">
	import { Dot } from '$lib/ui/dot';
	import type { LaunchItemProps } from '.';
	import { Button } from '$lib/ui/button';

	type $$Props = LaunchItemProps;
	export let name: $$Props['name'];
	export let selected: $$Props['selected'] = false;
	export let index: $$Props['index'];
	export let onClick: $$Props['onClick'];
</script>

<Button
	class="w-full px-8 py-7 flex items-center justify-between"
	variant="outline"
	on:click={() => onClick(index)}
>
	{name}
	{#if selected}
		<Dot />
	{/if}
</Button>
