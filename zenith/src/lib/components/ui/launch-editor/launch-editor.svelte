<script lang="ts">
	import * as Accordion from '$lib/components/ui/accordion';
	import { Switch } from "$lib/components/ui/switch";
	import { Label } from "$lib/components/ui/label";
	import type { LaunchEditorProps } from '.';

	type $$Props = LaunchEditorProps;
	export let subsystems: $$Props['subsystems'];
	console.log(subsystems);
</script>

<div class="w-full h-full overflow-y-scroll">
	Launch Editor
	<Accordion.Root class="w-full">
		{#each subsystems as subsystem}
			<Accordion.Item value={subsystem.name}>
				<Accordion.Trigger>{subsystem.name}</Accordion.Trigger>
				<Accordion.Content>
					<ul>
						{#each subsystem.packages.flatMap((pkg) => pkg.executables.map((exec) => `${pkg.name}::${exec}`) ) as pkgExecName}
							<li class="flex flex-wrap gap-4">
								<Label>{pkgExecName}</Label>
								<Switch onCheckedChange={console.log} id={pkgExecName} />
							</li>
						{/each}
					</ul>
				</Accordion.Content>
			</Accordion.Item>
		{/each}
	</Accordion.Root>
</div>
