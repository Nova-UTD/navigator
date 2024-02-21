<script lang="ts">
	import { Dot } from '$lib/components/ui/dot';
	import type { LaunchItemProps } from '.';

	type $$Props = LaunchItemProps;
	export let name: $$Props['name'];
	export let selected: $$Props['selected'] = false;
</script>

<div
	class="px-9 py-4 border-x border-b border-2 border-border flex items-center justify-between hover:bg-border cursor-pointer"
>
	{name}
	{#if selected}
		<Dot />
	{/if}
</div>
