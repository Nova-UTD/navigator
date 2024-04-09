<script lang="ts">
	import * as Accordion from '$lib/ui/accordion';
	import { Switch } from '$lib/ui/switch';
	import { Label } from '$lib/ui/label';
	import type { LaunchEditorProps } from '.';
	import { LazyLoadedList } from '$lib/ui/lazy-loaded-list';
	import { isNodeEqual } from '$lib/utils';

	type $$Props = LaunchEditorProps;
	export let subsystems: $$Props['subsystems'];
	export let activeNodes: $$Props['activeNodes'];
	export let onNodeChanged: $$Props['onNodeChanged'];
</script>

<Accordion.Root class="w-full">
	{#each subsystems as subsystem}
		<Accordion.Item value={subsystem.name}>
			<Accordion.Trigger>{subsystem.name}</Accordion.Trigger>
			<Accordion.Content>
				<LazyLoadedList
					ulClasses="overflow-x-hidden"
					items={subsystem.packages.flatMap((pkg) =>
						pkg.executables.map((exec) => ({ package: pkg.name, executable: exec }))
					)}
					itemsPerChunk={20}
					let:item={node}
				>
					<li class="flex flex-wrap gap-4">
						<Label>{`${node.package}::${node.executable}`}</Label>
						<Switch
							checked={activeNodes.some((n) => isNodeEqual(node, n))}
							onCheckedChange={(checked) => onNodeChanged(node, checked)}
							id={`${node.package}::${node.executable}`}
						/>
					</li>
				</LazyLoadedList>
			</Accordion.Content>
		</Accordion.Item>
	{/each}
</Accordion.Root>
