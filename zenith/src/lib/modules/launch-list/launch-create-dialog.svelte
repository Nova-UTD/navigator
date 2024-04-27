<script lang="ts">
	import * as Dialog from '$lib/ui/dialog';
	import { Label } from '$lib/ui/label';
	import { Input } from '$lib/ui/input';
	import { Button } from '$lib/ui/button';
	import { createLaunch } from '$lib/stores/launchStore';
	import { seqID } from '$lib/utils';
	import XIcon from 'lucide-svelte/icons/x';

	let open = false;
	let name = '';
	// TODO(gekevin): add actual file handling instead of string path
	let filename = `launch.${seqID()}.launch`;

	$: name !== '' && (filename = `launch.${name}.py`);

	let showError = false;
	let emptyError = false;

	$: emptyError = name === '' || filename === '';

	function changeName(event: Event) {
		showError = false;
		const target = event.target as HTMLInputElement;
		name = target.value;
	}

	function changePath(event: Event) {
		showError = false;
		const target = event.target as HTMLInputElement;
		filename = target.value;
	}
</script>

<Dialog.Root bind:open>
	<Dialog.Trigger><slot /></Dialog.Trigger>
	<Dialog.Content class="text-white sm:max-w-[425px]">
		<Dialog.Header>
			<Dialog.Title>Create Launch File</Dialog.Title>
		</Dialog.Header>
		<div class="grid gap-4 py-4">
			<div class="grid grid-cols-4 items-center gap-4">
				<Label for="name" class="text-right">Name</Label>
				<Input id="name" autofocus on:input={changeName} value={name} class="col-span-3" />
			</div>
			<div class="grid grid-cols-4 items-center gap-4">
				<Label for="filename" class="text-right">Filename (exclude path)</Label>
				<Input id="filename" on:input={changePath} value={filename} class="col-span-3" />
			</div>
		</div>
		{#if showError && emptyError}
			<div class="flex items-center gap-2 text-xs text-red-500">
				<XIcon class="w-8 h-8" />
				<p>Name and filename cannot be empty.</p>
			</div>
		{/if}
		<Dialog.Footer>
			<Button
				on:click={() => {
					if (emptyError) {
						showError = true;
						return;
					}
					open = false;
					createLaunch(name, filename);
				}}
				type="submit"
				variant="secondary">Create</Button
			>
		</Dialog.Footer>
	</Dialog.Content>
</Dialog.Root>
