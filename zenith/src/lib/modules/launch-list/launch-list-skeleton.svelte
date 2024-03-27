<script lang="ts">
	import { Skeleton } from '$lib/ui/skeleton';
</script>

<ul>
	{#each Array.from({ length: 3 }) as _}
		<li class="mb-2">
			<Skeleton
				class="w-full px-9 py-7 border-x border-b border-2 border-border flex items-center justify-between hover:bg-border cursor-pointer"
			/>
		</li>
	{/each}
</ul>
