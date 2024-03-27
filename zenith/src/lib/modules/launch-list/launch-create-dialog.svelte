<script lang="ts">
	import * as Dialog from '$lib/ui/dialog';
	import { Label } from '$lib/ui/label';
	import { Input } from '$lib/ui/input';
	import { Button } from '$lib/ui/button';
	import { createLaunch } from '$lib/stores/launchStore';

	let open = false;
	let name = '';
	// TODO(gekevin): add actual file handling instead of string path
	let path = '/';

	function changeName(event: Event) {
		const target = event.target as HTMLInputElement;
		name = target.value;
	}

	function changePath(event: Event) {
		const target = event.target as HTMLInputElement;
		path = target.value;
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
				<Label for="name" class="text-right">Directory Path</Label>
				<Input id="name" on:input={changePath} value={path} class="col-span-3" />
			</div>
		</div>
		<Dialog.Footer>
			<Button
				on:click={() => {
					open = false;
					createLaunch(name, path);
				}}
				type="submit"
				variant="secondary">Create</Button
			>
		</Dialog.Footer>
	</Dialog.Content>
</Dialog.Root>
